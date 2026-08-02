/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file security_service.hpp
 * @brief xbot SecurityService (id 14): anti-theft escalation ladder, owner identity, PIN.
 *
 * Firmware-autonomous anti-theft (issue #123): once armed (owner opt-in, requires a PIN)
 * and undocked, a LIFT/LIFT_MULTIPLE emergency sustained past "Trigger Delay S" (default
 * 3 s - so a bump never alarms) starts a 3-stage ladder, all driven by elapsed time since
 * the trigger:
 *   Stage 1 WARNING   0s .. "Deterrent Delay S"           procedural chirps, rising cadence
 *   Stage 2 DETERRENT ..  "Siren Delay S"                 spoken deterrent clip, looped
 *   Stage 3 SIREN      "Siren Delay S"..                  full-scale siren sweep, alternating
 *                                                          with the deterrent + owner clips;
 *                                                          duty-cycles after "Siren Max S"
 * The four delays are xbot inputs (ROS/field-tunable live, no reflash) with compile-time
 * defaults; see the constants block at the top of security_service.cpp for those defaults
 * and every other ladder/PIN timing this file uses - ALL of them are first guesses that
 * NEED FIELD TUNING on a real robot.
 *
 * A ROS geofence/kidnap verdict can also drive the ladder directly via the unauthenticated
 * `Trigger Alarm` RPC - deliberately unauthenticated because escalating is never a security
 * downgrade. It only ever raises the ladder, never lowers it, and only does anything at all
 * once the owner has armed the feature.
 *
 * Docked lift never alarms: while the charger adapter is present (PowerService's debounced
 * dock-presence signal - see PowerService::IsAdapterPresent()), the LIFT-sustain check is
 * skipped entirely, so cleaning/servicing the mower on its dock is silent. This does NOT
 * gate `Trigger Alarm` - a real geofence violation cannot happen while genuinely docked
 * anyway, so there is nothing to protect against there.
 *
 * Persistence (survives a power cycle, e.g. a thief pulling the battery to silence it):
 * `armed`, the alarm `latch`, the PIN hash/salt and the failed-attempt counter all live in
 * one small versioned struct at /cfg/security_state.bin (see security_service.cpp). The
 * latch is the important one: it is set the instant the ladder first leaves ARMED, written
 * synchronously right then (the one flash write in this file that is NOT rate-limited to an
 * edge, because losing it to a race would defeat the whole feature), and never cleared
 * except by a correctly-PIN'd `Clear Alarm`. OnStart() checks it before anything else: if
 * it is still set, the ladder skips straight to SIREN and speaks the owner clip, instead of
 * re-running the 3 s/10 s/60 s escalation from scratch. Every other persisted field is only
 * written on its own edge (PIN change, arm toggle, failed-attempt count, latch clear) -
 * never once per tick - per the correctness bar on flash write-wear.
 *
 * PIN storage: NOT plaintext. A PIN (4-8 digits) is hashed as a salted, doubly-mixed CRC32
 * (littlefs's own lfs_crc(), already linked in - no crypto library added) - see HashPin()
 * in the .cpp. This is explicitly NOT a KDF: CRC32 is linear and technically invertible, so
 * it does not resist a determined attacker with the salt and unlimited offline attempts.
 * The threat model it is sized for is a casual flash dump: no plaintext PIN sits in the
 * image, and combined with the escalating wrong-PIN lockout (also in the .cpp) there is no
 * practical way to brute-force an 8-digit PIN against the live device. If this ever needs
 * to resist more than that, swap HashPin() for a real KDF; nothing else in this file
 * assumes the specific hash. The salt itself is generated from boot-relative timing jitter
 * (getTimeMicros() + the free-running ChibiOS realtime counter, both already available, no
 * new peripheral driver) - not a CSPRNG, but not attacker-predictable from outside either.
 *
 * /cfg is deliberately outside FilesystemService's RPC sandbox (rooted at /user -
 * BuildSafePath() there always prefixes every path with "/user", so nothing reachable
 * through ListFiles/RemoveFile/AddFileChunk can ever resolve to anything under /cfg,
 * structurally, not just by validation). Owner identity lives at /cfg/owner/owner.txt (free
 * text, written directly by the PIN-gated `Set Owner Info` RPC) and /cfg/owner/owner.wav
 * (the ROS-synthesized clip). Since ROS can only reach /user, owner.wav is uploaded there
 * first, at a fixed staging name, and `Set Owner Info` - already PIN-gated - promotes it
 * into /cfg/owner by a plain in-firmware file copy (see PromoteOwnerWav()). Once promoted,
 * nothing RPC-driven can read, overwrite or delete it: the copy is one-directional and only
 * firmware code (this file) ever opens a path under /cfg.
 *
 * SHIP DISARMED / safe default: `Set Armed(Armed=1)` requires a valid PIN, full stop - so
 * with no PIN configured (the out-of-the-box state), arming is unconditionally rejected
 * (ERR_LOCKED). There is therefore no reachable state, ever, that the owner cannot get out
 * of: either no PIN exists and the ladder can never start, or a PIN exists and that same
 * PIN always disarms/clears it.
 */

#ifndef SECURITY_SERVICE_HPP
#define SECURITY_SERVICE_HPP

#include <ch.h>

#include <SecurityServiceBase.hpp>

using namespace xbot::service;

class SecurityService : public SecurityServiceBase {
 public:
  explicit SecurityService(uint16_t service_id) : SecurityServiceBase(service_id, wa, sizeof(wa)) {
  }

 protected:
  bool OnStart() override;
  uint32_t OnLoop(uint32_t now_micros, uint32_t last_tick_micros) override;

  void OnTriggerDelaySChanged(const uint8_t& new_value) override;
  void OnDeterrentDelaySChanged(const uint8_t& new_value) override;
  void OnSirenDelaySChanged(const uint16_t& new_value) override;
  void OnSirenMaxSChanged(const uint16_t& new_value) override;

  void RPCSetArmed(uint16_t call_id, const char* Pin, uint32_t PinLen, uint8_t Armed) override;
  void RPCClearAlarm(uint16_t call_id, const char* Pin, uint32_t PinLen) override;
  void RPCTriggerAlarm(uint16_t call_id, uint8_t Stage) override;
  void RPCSetOwnerInfo(uint16_t call_id, const char* Pin, uint32_t PinLen, const char* NewPin, uint32_t NewPinLen,
                       const char* Text, uint32_t TextLen) override;

 public:
  // Sizes of the (salted, hashed) PIN storage - public so the free helper functions in
  // security_service.cpp (HashPin(), GenerateSalt(), the persisted-state struct) can size
  // their buffers off the same single definition instead of a second copy.
  static constexpr size_t kPinHashBytes = 4;
  static constexpr size_t kPinSaltBytes = 8;

 private:
  // Internal PIN-check outcome. Distinct from the wire SecResult enum so the lockout /
  // bootstrap branches below read clearly; mapped to SecResult at each RPC boundary.
  enum class AuthResult : uint8_t { OK, WRONG_PIN, LOCKED_OUT, NO_PIN_SET };

  AuthResult CheckPin(const char* pin, uint32_t pin_len);
  void RegisterFailedAttempt(uint32_t now_micros);
  void RegisterSuccessfulAuth();
  bool SetPin(const char* new_pin, uint32_t new_pin_len);  ///< also used for the first-ever (bootstrap) PIN
  static uint8_t AuthResultToWire(AuthResult auth);        ///< maps to the wire SecResult enum

  void LoadPersisted();
  void Persist();

  void SetStage(AlarmState new_stage,
                uint32_t now_micros);  ///< the only place stage_ changes; handles all side effects
  void TriggerLadder(uint32_t now_micros, AlarmState requested_stage);  ///< escalate only, never downgrades
  AlarmState ComputeStageForElapsed(uint32_t elapsed_ms) const;
  void UpdateLadder(uint32_t now_micros);
  void CheckLiftTrigger(uint32_t now_micros);
  void DriveStageSounds(uint32_t now_micros, uint32_t elapsed_ms);
  void SendStatus();

  bool WriteOwnerText(const char* text, uint32_t text_len);
  void PromoteOwnerWav();  ///< no-op if nothing has been staged via the FS service; see file header

  [[nodiscard]] bool IsDocked() const;
  [[nodiscard]] bool ArmGateActive() const;  ///< armed_ && !docked; gates the LIFT-sustain trigger only

  bool ever_started_ = false;  ///< OnStart() re-runs on every xbot CLAIM; see AudioService for why this guard exists
  bool owner_set_ = false;     ///< derived from /cfg/owner/owner.txt existing, checked once at boot

  // --- persisted across power cycles (see Persist()/LoadPersisted(), /cfg/security_state.bin) ---
  bool armed_ = false;
  bool alarm_latched_ = false;
  bool pin_set_ = false;
  uint8_t pin_hash_[kPinHashBytes]{};
  uint8_t pin_salt_[kPinSaltBytes]{};
  uint16_t failed_attempts_ = 0;

  // --- ladder state. Service-thread only: RPC handlers and OnLoop both run there (same as
  // AudioService), so none of this needs a mutex. ---
  AlarmState stage_ = AlarmState::DISARMED;
  AlarmState forced_min_stage_ = AlarmState::WARNING;  ///< floor set by Trigger Alarm RPC; never lowered mid-cycle
  uint32_t trigger_micros_ = 0;  ///< when the current ladder cycle started; stage boundaries are offsets from this

  bool lift_active_ = false;
  uint32_t lift_since_micros_ = 0;

  bool lockout_active_ = false;
  uint32_t lockout_until_micros_ = 0;

  // Stage-local sound-cycle sub-state, see DriveStageSounds().
  uint32_t last_warn_chirp_micros_ = 0;
  uint32_t last_deterrent_micros_ = 0;
  uint32_t siren_phase_start_micros_ = 0;  ///< when SIREN was entered; drives the continuous->burst switch
  uint32_t siren_subphase_start_micros_ =
      0;  ///< 0 = "start a fresh siren-tone opener"; drives tone/deterrent/owner rotation
  uint8_t siren_subphase_index_ = 0;

  // --- ladder timing, xbot inputs (live-tunable; compile-time defaults + validated in the
  // .cpp constants block, set once at boot in OnStart()) ---
  uint8_t trigger_delay_s_ = 0;
  uint8_t deterrent_delay_s_ = 0;
  uint16_t siren_delay_s_ = 0;
  uint16_t siren_max_s_ = 0;

  ServiceSchedule status_schedule_{*this, 1'000'000,
                                   XBOT_FUNCTION_FOR_METHOD(SecurityService, &SecurityService::SendStatus, this)};

  THD_WORKING_AREA(wa, 3072){};
};

#endif  // SECURITY_SERVICE_HPP
