/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file sound_service.hpp
 * @brief Plays WAV files from LittleFS (/user/audio/<n>.wav) through the I2S6Audio driver.
 *
 * Unlike the other entries in src/services, this is a plain ChibiOS thread - not an xbot
 * Service - since it has no RPC surface and needs no service_id. It owns a small comms-style
 * thread (see drivers/ui/yf_cover_ui for the pattern this mirrors) that:
 *  - polls EmergencyService for reason-mask edges (0->nonzero / nonzero->0) and plays the
 *    matching V1 sound-map track (see SoundEvent below).
 *
 * The track numbering is ported 1:1 from OpenMower V1's sound map (see SoundEvent). Only the
 * emergency-triggered events (STOP/LIFT/other -> active, and -> cleared) are wired up so far;
 * the rest (ROS/GPS/rain/high-level-state driven) are listed for completeness but not yet
 * triggered from anywhere - see the "TODO(v1-sound-map)" markers in sound_service.cpp.
 */

#ifndef SOUND_SERVICE_HPP
#define SOUND_SERVICE_HPP

#include <ch.h>

#include <cstdint>

/**
 * @brief OpenMower V1 sound-map events, ported 1:1. The numeric value is the LittleFS track
 * number: event N plays /user/audio/<N>.wav.
 */
enum class SoundEvent : uint8_t {
  kBootGreeting = 1,       ///< Played once at boot.
  kOpenMowerInitOk = 2,    ///< OpenMower-side init completed successfully.
  kRosInit = 3,            ///< ROS side is initializing.
  kMapRecordStart = 4,     ///< Map recording started.
  kRtkWait = 5,            ///< Waiting for RTK fix.
  kEmergencyStop = 8,      ///< Emergency became active: STOP button.
  kEmergencyLift = 9,      ///< Emergency became active: LIFT / LIFT_MULTIPLE.
  kRain = 10,              ///< Rain detected.
  kMowDoneDock = 11,       ///< Mowing finished / docking.
  kAutonomousStart = 12,   ///< Autonomous mowing started.
  kRosStartupOk = 16,      ///< ROS stack finished starting up successfully.
  kRosStopped = 17,        ///< ROS stack stopped.
  kImuInitFailed = 19,     ///< IMU init failed.
  kGpsPoor = 20,           ///< GPS reception poor.
  kGpsModerate = 21,       ///< GPS reception moderate.
  kGpsGood = 22,           ///< GPS reception good.
  kEmergencyCleared = 23,  ///< Emergency reasons all cleared (nonzero -> 0).
  kEmergencyOther = 24,    ///< Emergency became active: any other reason (COLLISION/TIMEOUT/...).
};

class SoundService {
 public:
  /**
   * @brief Ensure /user/audio exists, init the I2S6 driver, and start the polling thread.
   * Call after FilesystemService (or at least InitFS()) and EmergencyService are up.
   */
  void Start();

  /**
   * @brief Stream /user/audio/<n>.wav (PCM, mono, 16-bit, i2s6_audio::kSampleRateHz) out via the
   * I2S6 driver. Reads the file in small chunks rather than loading it into RAM. Logs a warning
   * and does nothing if the file is missing or its format doesn't match those assumptions.
   */
  void PlayTrack(uint8_t n);

  /** @brief PlayTrack(), addressed by SoundEvent instead of a raw track number. */
  void PlayEvent(SoundEvent event);

  /** @brief Playback volume: 0 (mute) .. 256 (unity gain). Forwarded to the I2S6 driver. */
  void SetVolume(uint16_t volume);

 private:
  static void ThreadHelper(void* instance);
  void ThreadFunc();

  thread_t* thread_ = nullptr;
  THD_WORKING_AREA(wa_, 1024);
};

#endif  // SOUND_SERVICE_HPP
