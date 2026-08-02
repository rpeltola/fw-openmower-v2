# fw-openmower-v2 — STM32H723 firmware

## Branch model (MOST IMPORTANT — read before touching any code)

**`personal` is the branch we always use with the ROS2 stack. It is what's built and
flashed on the real mower.** Any investigation of "what the firmware actually does on
the robot" MUST be done against `personal`, not `main`.

- **`main`** mirrors **upstream** — do NOT assume it reflects our deployed behavior.
- **`personal`** is our **integration branch**: features branch off upstream, then
  merge `--no-ff` into `personal`. This is the trunk for our own work + the ROS2 side.
- Develop features in worktrees under the workspace root's
  `.worktrees/fw-openmower-v2/<feature>` — OUTSIDE this repo (changed 2026-07-25; nested
  inside it, this CLAUDE.md loaded twice per session — see the workspace `CLAUDE.md`) —
  then merge into `personal`. Create one with
  `git worktree add ../.worktrees/fw-openmower-v2/<feature> -b feature/<name> <base>`.
  Never branch-switch the base checkout (multiple Claude windows share it).

If you need "the firmware running on the mower," check out / read `personal`.

## Drive control (diff_drive) — current state

- `diff_drive_service` converts the Control Twist → per-wheel setpoints → ESC command.
- **`main`** is open-loop: it treats the Twist components *directly as duty*
  (`duty = 0.5·WheelDistance·ω` for rotation) with no closed loop and a stale
  `TODO: implement xESC speed control`.
- **`personal`** has the **duty_loop** speed control implemented (that TODO is gone), and
  its Control Twist is a **physical velocity** — `linear.x` m/s, `angular.z` rad/s — not a
  duty. A `Control Mode` register selects `0 = open-loop duty` / `1 = duty_loop`.
- **duty_loop closes on the ESC's own eRPM, NOT on the tacho.** The tacho ticks only 6
  times per electrical revolution, far too coarse to close a loop at walking pace;
  `eRPM/10 / ticks-per-meter` is m/s with the pole pairs cancelling. Odometry (Actual
  Twist / Wheel Ticks) deliberately *stays* tacho-based — do not "unify" the two.
- Tuning surface is larger than a Kp/Ki pair: registers `Loop Kp/Ki/Max Output/Slew/Ks/Kv`
  and `Accel Limit` (service defs v3, registers 0–9), plus a live **`Loop Tuning` input**
  that overrides gains without restarting the service — that is what the tuning harness
  sweeps with. Precedence: live input → register → built-in. Feedforward is
  `ks·sign(v) + kv·v`; `ks` (breakaway) is still 0 pending characterization on the robot.
- **duty_loop IS enabled on the running mower** — the live `~/params/settings.yaml` has
  `control_mode: 1`. (Open question through 2026-07; settled 2026-08-01.)
- Stale ESC telemetry stops the wheels. duty_loop's command is computed from telemetry, so
  a receive-only ESC failure would otherwise re-send the last PI output forever while the
  twist keeps arriving. `tick()` treats telemetry older than `kEscTelemetryTimeoutUs` as a
  stop via `ResetControlState()` (so the setpoint ramps from standstill on recovery) and
  invalidates the odometry window in **both** modes, because an ESC that reboots during the
  gap restarts its tacho at 0.
- Low-speed behaviour is an **observability** problem, not a tuning one: the xESC runs
  sensorless FOC, so back-EMF → 0 near standstill and eRPM is blind exactly where the mower
  needs it. Read `openmower_knowledgebase/low-speed-drive-control-sensorless-erpm.md`
  before touching gains, and `drive-twist-jitter-esc-desync-50hz.md` for the odometry-dt
  hazard at 50 Hz.

## Known firmware defects

Whole-firmware audit (2026-08-01): `openmower_knowledgebase/fw-v2-full-audit-20260801.md`.
Unfixed items that will bite anyone working nearby:

- Debug TCP 65102/65103/65104 are open in **release** builds, and raw mode makes
  `VescDriver::SetDuty()` a no-op — while a client is connected the emergency zero-duty
  write never reaches the motors.
- A five-byte `RESET` UDP datagram reboots the MCU into the bootloader, unauthenticated.
- `YFCoverUI::CobsDecode()` has no output bound; a 32-byte frame writes 7 bytes past
  `decode_buf_`.
- No IWDG/WWDG anywhere, and several unbounded init/recovery loops can hang a service
  thread with no watchdog to recover it.
- Audio: `dma_buffer_`'s D-cache-vs-BDMA coherency (issue #119) is fixed on `personal`
  (`cacheBufferFlush()` after every half-buffer write) — build-verified only, NOT YET
  bench/HW-validated.

## Build / flash

See `openmower_knowledgebase` and memory (flashing playbook): build our changes, name
bins `openmower-<Robot>-<feature>.bin` in the mower's `~/fw-flash`, flash via
`flash-firmware.sh`. Do not flash unless explicitly told.
