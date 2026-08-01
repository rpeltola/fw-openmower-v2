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
 * @brief xbot AudioService (id 13): named sounds, procedural tones, playback arbitration.
 *
 * Successor of the plain-thread SoundService: the same I2S6/WAV playback pipeline, now a
 * real xbot service so ROS can trigger sounds (Play Local / Play Tone RPCs) and configure
 * volume/quiet mode (inputs), with a priority-class model arbitrating who owns the single
 * playback voice.
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
 * ignore quiet mode.
 *
 * All playback starts on the service thread (RPC handlers and OnLoop both run there).
 * Cross-thread callers (CoverUI comms thread, power service) use RequestTone()/
 * RequestNamed(), a mutex-guarded 1-deep mailbox drained by OnLoop.
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

  void OnMasterVolumeChanged(const uint16_t& new_value) override;
  void OnQuietModeChanged(const uint8_t& new_value) override;

 private:
  struct Request {
    bool valid = false;
    bool is_tone = false;
    uint8_t audio_class = 0;
    uint8_t pattern = 0;
    char name[64]{};
  };

  // Returns an AudioResult as its wire byte. All Start* run on the service thread only.
  uint8_t StartNamed(const char* name, size_t name_len, AudioClass audio_class);
  uint8_t StartTone(TonePattern pattern, AudioClass audio_class, uint16_t freq, uint16_t duration_ms, uint8_t count);

  bool ArbitrateStart(AudioClass audio_class);  ///< false = keep current sound, drop the request
  void ApplyClassVolume(AudioClass audio_class);
  bool PlayPath(const char* path);  ///< open + validate a WAV and start streaming it
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

  ServiceSchedule status_schedule_{*this, 1'000'000,
                                   XBOT_FUNCTION_FOR_METHOD(AudioService, &AudioService::SendStatus, this)};

  THD_WORKING_AREA(wa, 4096){};
};

#endif  // AUDIO_SERVICE_HPP
