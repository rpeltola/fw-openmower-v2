/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file stream_source.hpp
 * @brief Jitter-buffered ROS PCM stream, exposed as an i2s6_audio SampleSource.
 *
 * Backs the AudioService "Stream Frame" input / Start Stream / Stop Stream RPCs (issue
 * #122): ROS pushes 16 kHz mono PCM16 frames, this module conceals lost frames (gap in
 * seq -> silence, never a retransmit request) into a ring buffer, and the I2S6 driver
 * pulls from it like it would a file or a tone program.
 *
 * Threading mirrors tone_source.cpp / audio_service.cpp's playback_mutex_: Open(),
 * PushFrame() and Close() run on the AudioService thread only (never concurrently with
 * each other), FeedStream() runs on the I2S6 driver's own feeder thread. A mutex
 * serializes the ring between the two. As with playback_mutex_, never call into
 * i2s6_audio while holding it.
 *
 * The ring buffer itself is heap-allocated (D1 AXI SRAM - see stream_source.cpp) on
 * Open() and freed on Release(), rather than a static 32 KiB .bss array: SRAM4 (where
 * i2s6_audio's own DMA buffer lives) is far too small for it, and the D1 heap only needs
 * to carry the cost while a stream is actually open.
 */

#ifndef STREAM_SOURCE_HPP
#define STREAM_SOURCE_HPP

#include <cstddef>
#include <cstdint>

namespace stream {

/// Nominal (design-target) mono samples per wire frame: 40 ms @ 16 kHz, matching the ROS
/// sender's pacing (issue #122). This is NOT a hard wire size - the generated dispatcher
/// does not enforce one (see audio_service.cpp's OnStreamFrameChanged()), so a frame that
/// legitimately arrives with a different sample count still plays correctly. Used here only
/// as the target for concealment (how much silence one lost frame is worth) and the
/// prebuffer gate (200 ms = 5 nominal frames).
inline constexpr size_t kNominalFrameSamples = 640;

/**
 * @brief Claim the stream slot: allocates the ring buffer and resets seq tracking/stats.
 *
 * AudioService thread only. Safe to call again while already open (no-op, returns true).
 * @return false if the ring allocation failed - caller must not proceed with the stream.
 */
bool Open();

/**
 * @brief Release the stream slot: frees the ring buffer and clears all state.
 *
 * Must only be called once no FeedStream() call can still be in flight against the ring
 * - i.e. after xbot::driver::audio::Stop() has returned (it blocks until any in-flight
 * refill finishes, same guarantee audio_service.cpp's CloseStream() relies on), or once
 * the driver has muted itself following FeedStream()'s own end-of-stream short read (see
 * Close()). AudioService thread only. Safe to call when not open (no-op).
 */
void Release();

/**
 * @brief Push one wire frame's samples into the ring, concealing any gap in seq.
 *
 * seq is a wrapping uint16_t frame counter; the gap is computed with wraparound-safe
 * modular arithmetic, not a plain `>` comparison. A gap up to kMaxConcealFrames is filled
 * with silence (lost-packet concealment, per issue #122 - no retransmit is ever
 * requested); a larger jump (stream restart, or seq we can't make sense of) just
 * resyncs instead of manufacturing a long silence. Ignored once Close() has been called.
 * Never blocks: drops (and counts) the whole frame if the ring has no room for it -
 * chosen over a partial write so a dropped frame can't corrupt the framing of the one
 * after it.
 *
 * AudioService thread only.
 */
void PushFrame(uint16_t seq, const int16_t* samples, size_t n_samples);

/**
 * @brief Mark the stream as ending: no more input is coming.
 *
 * Does not touch hardware or free anything by itself - it only changes what FeedStream()
 * does once the ring runs dry (see its own comment). An immediate cut (Stop Stream
 * flush=1) is done by the caller via xbot::driver::audio::Stop() instead of through here.
 * AudioService thread only.
 */
void Close();

/// Samples currently buffered, for the "Buffer Free Bytes" status output.
size_t BufferedSamples();

/**
 * @brief True once at least 200 ms (5 frames) is buffered.
 *
 * AudioService::OnLoop() polls this while a stream is open but not yet playing: once true,
 * it calls xbot::driver::audio::Play(&FeedStream) to start the DMA. Buffering before the
 * first Play() call (rather than starting the instant a frame arrives) absorbs ordinary
 * jitter in the first few packets so playback doesn't immediately underrun.
 */
bool PrebufferReady();

/// Free space in the ring, in bytes, for the "Buffer Free Bytes" status output.
uint16_t FreeBytes();

/// Cumulative count of distinct underrun episodes, for the "Underruns" status output.
uint16_t UnderrunEvents();

/**
 * @brief Test-and-clear: true once the current underrun episode has run past 500 ms.
 *
 * AudioService::OnLoop() polls this once per tick to decide whether to auto-stop the
 * stream. Safe to call from the AudioService thread regardless of what FeedStream() is
 * doing concurrently on the feeder thread (backed by an atomic).
 */
bool TakeUnderrunExceeded();

/**
 * @brief i2s6_audio SampleSource: pop up to n_samples samples for the feeder thread.
 *
 * Never blocks, and - unlike FeedFromFile()/FeedTone() - does not treat "not enough data
 * right now" as end-of-stream: a ring that has run dry is padded with silence and counted
 * as an underrun so a transient stall (network jitter, a slow ROS sender) can't be
 * mistaken by the driver for the track finishing. The only time this returns fewer than
 * n_samples is once Close() has been called AND the ring has fully drained - that is the
 * deliberate "let the buffered audio play out" end-of-stream signal, at which point the
 * driver's own end-of-stream handling (silence-pad the tail, then auto-mute) takes over,
 * exactly as it would for an exhausted file or tone program.
 */
size_t FeedStream(int16_t* dst, size_t n_samples);

}  // namespace stream

#endif  // STREAM_SOURCE_HPP
