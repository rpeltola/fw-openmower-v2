/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file audio_service.hpp
 * @brief xbot AudioService (id 13): named sounds, procedural tones, ROS PCM streaming,
 *        playback arbitration.
 *
 * Successor of the plain-thread SoundService: the same I2S6/WAV playback pipeline, now a
 * real xbot service so ROS can trigger sounds (Play Local / Play Tone RPCs), stream
 * arbitrary 16 kHz mono PCM16 audio (Start Stream / Stop Stream RPCs + the Stream Frame
 * input, issue #122), and configure volume/quiet mode (inputs) - with a priority-class
 * model arbitrating who owns the single playback voice.
 *
 * Sound resolution for a named sound (all callers, including the built-in triggers):
 *   1. /user/audio/<name>.wav            - user override uploaded via FilesystemService
 *   2. /user/audio/<n>.wav               - legacy V1 numbered track, via the alias table
 *   3. built-in tone program             - for the UI beep names (see kToneDefaults)
 * so every sound can be personalized by uploading a file, and "reset to default" is just
 * deleting it.
 *
 * Priority classes (AudioClass, from the service definition): ALARM > SAFETY > UI >
 * STATUS > AMBIENT. A request preempts anything of equal or lower class and is dropped
 * (ERR_DROPPED) while something higher is playing. SAFETY/ALARM get a volume floor and
 * ignore quiet mode. A stream claims its class the moment Start Stream succeeds (before
 * any audio actually plays - see stream_playing_), so it can't be silently stepped on by
 * a lower-or-equal request during its 200 ms prebuffer window; StartNamed()/StartTone()
 * release a stream slot they preempt (ReleaseStreamSlot()) so the single slot can never be
 * left stuck claimed by a stream that lost the voice to something else.
 *
 * All playback starts on the service thread (RPC handlers and OnLoop both run there).
 * Cross-thread callers (CoverUI comms thread, power service) use RequestTone()/
 * RequestNamed(), a mutex-guarded 1-deep mailbox drained by OnLoop. The stream's ring
 * buffer (stream_source.{cpp,hpp}) is the exception worth knowing about: OnStreamFrameChanged()
 * also runs on the service thread (it's just another xbot input handler), so pushing into the
 * ring needs no mailbox - only its consumer, the I2S6 driver's feeder thread, is cross-thread,
 * and that's guarded the same way tone_source.cpp/playback_mutex_ already are.
 */

#ifndef AUDIO_SERVICE_HPP
#define AUDIO_SERVICE_HPP

#include <ch.h>
#include <etl/atomic.h>

#include <AudioServiceBase.hpp>

using namespace xbot::service;

class AudioService : public AudioServiceBase {
 public:
  explicit AudioService(uint16_t service_id) : AudioServiceBase(service_id, wa, sizeof(wa)) {
  }

  /**
   * @brief Thread-safe fire-and-forget sound requests, callable from any thread.
   *
   * Stored in a 1-deep mailbox (a newer request of equal or higher class replaces a
   * pending one) and started on the service thread within one OnLoop period (~10 ms).
   */
  void RequestTone(TonePattern pattern, AudioClass audio_class);
  void RequestNamed(const char* name, AudioClass audio_class);

 protected:
  bool OnStart() override;
  uint32_t OnLoop(uint32_t now_micros, uint32_t last_tick_micros) override;

  void RPCPlayLocal(uint16_t call_id, const char* Name, uint32_t NameLen, uint8_t Class) override;
  void RPCPlayTone(uint16_t call_id, uint8_t Pattern, uint8_t Class, uint16_t Freq, uint16_t DurationMs,
                   uint8_t Count) override;
  void RPCStartStream(uint16_t call_id, uint8_t Class) override;
  void RPCStopStream(uint16_t call_id, uint8_t Flush) override;

  void OnMasterVolumeChanged(const uint16_t& new_value) override;
  void OnQuietModeChanged(const uint8_t& new_value) override;
  void OnStreamFrameChanged(const uint8_t* new_value, uint32_t length) override;

 private:
  struct Request {
    bool valid = false;
    bool is_tone = false;
    uint8_t audio_class = 0;
    uint8_t pattern = 0;
    char name[64]{};
  };

  // Returns an AudioResult as its wire byte. All Start*/Stop* run on the service thread only.
  uint8_t StartNamed(const char* name, size_t name_len, AudioClass audio_class);
  uint8_t StartTone(TonePattern pattern, AudioClass audio_class, uint16_t freq, uint16_t duration_ms, uint8_t count);
  uint8_t StartStream(AudioClass audio_class);
  uint8_t StopStream(bool flush);

  bool ArbitrateStart(AudioClass audio_class);        ///< false = keep current sound, drop the request
  uint16_t ApplyClassVolume(AudioClass audio_class);  ///< returns the volume it resolved and set
  bool PlayPath(const char* path);                    ///< open + validate a WAV and start streaming it
  void ReleaseStreamSlot();                           ///< tears down an open stream; no-op if none is open
  void CheckEmergencyEdges();
  void DrainPending();
  void SendStatus();

  MUTEX_DECL(req_mtx_);
  Request pending_{};

  uint8_t playing_class_ = 0;                 ///< AudioClass of the current sound, 0 when idle. Service thread only.
  etl::atomic<uint16_t> master_volume_{205};  ///< 0..256; SAFETY/ALARM floors override it
  etl::atomic<bool> quiet_mode_{false};

  bool driver_ok_ = false;
  bool ever_started_ = false;  ///< distinguishes power-on from a re-claim restart (see OnStart())
  bool boot_chime_played_ = false;
  bool grace_elapsed_ = false;  ///< latched once the startup grace expired (see OnLoop())
  uint32_t start_micros_ = 0;
  uint16_t last_reasons_ = 0;

  // Stream slot bookkeeping (issue #122). Service thread only, like every other playback state
  // above. stream_open_ is true from a successful StartStream() until the slot is released
  // (flush stop, drained stop, preemption, or underrun auto-stop); stream_playing_ only becomes
  // true once the 200 ms prebuffer fills and xbot::driver::audio::Play() is actually called -
  // see OnLoop(), which uses the gap between the two to avoid mistaking "still prebuffering" for
  // "finished".
  bool stream_open_ = false;
  bool stream_playing_ = false;

  ServiceSchedule status_schedule_{*this, 1'000'000,
                                   XBOT_FUNCTION_FOR_METHOD(AudioService, &AudioService::SendStatus, this)};

  THD_WORKING_AREA(wa, 4096){};
};

#endif  // AUDIO_SERVICE_HPP
