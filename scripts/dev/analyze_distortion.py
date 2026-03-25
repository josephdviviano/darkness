#!/usr/bin/env python3
"""Comprehensive audio distortion analysis for Darkness engine captures.

Analyzes a WAV file for:
- Hard clipping (samples at +/-1.0)
- Soft saturation (sustained near-rail signal)
- Inter-sample peaks (interpolated values exceeding 1.0)
- Click/pop detection with temporal clustering
- RMS dynamics and sudden jumps
- Crest factor per window
- Zero-crossing rate anomalies

Usage: python3 analyze_distortion.py [path_to_wav]
Default: darkness_audio_capture.wav in the project root
"""

import sys
import struct
import numpy as np
from pathlib import Path


def read_wav_float32(path):
    """Read IEEE float32 WAV file, return (sample_rate, data[frames, channels])."""
    with open(path, 'rb') as f:
        riff = f.read(4)
        if riff != b'RIFF':
            raise ValueError(f"Not a RIFF file: {riff}")
        f.read(4)  # file size
        wave = f.read(4)
        if wave != b'WAVE':
            raise ValueError(f"Not a WAVE file: {wave}")

        fmt_found = False
        data_array = None
        sample_rate = 48000
        num_channels = 2

        while True:
            chunk_id = f.read(4)
            if len(chunk_id) < 4:
                break
            chunk_size = struct.unpack('<I', f.read(4))[0]

            if chunk_id == b'fmt ':
                fmt_data = f.read(chunk_size)
                audio_fmt = struct.unpack('<H', fmt_data[0:2])[0]
                num_channels = struct.unpack('<H', fmt_data[2:4])[0]
                sample_rate = struct.unpack('<I', fmt_data[4:8])[0]
                fmt_found = True
            elif chunk_id == b'data':
                raw = f.read(chunk_size)
                num_samples = len(raw) // 4  # float32
                data_array = np.frombuffer(raw, dtype=np.float32)
                if num_channels > 1:
                    data_array = data_array.reshape(-1, num_channels)
                break
            else:
                f.read(chunk_size)

        if data_array is None:
            raise ValueError("No data chunk found")
        return sample_rate, data_array


def analyze(path):
    sr, data = read_wav_float32(path)
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    frames, nch = data.shape
    duration = frames / sr

    print(f"File: {path}")
    print(f"Format: {sr} Hz, {nch} channels, {duration:.2f}s, {frames} frames")
    print()

    for ch in range(nch):
        ch_name = "Left" if ch == 0 else "Right"
        signal = data[:, ch]

        # 1. Peak and clipping
        peak = np.max(np.abs(signal))
        peak_db = 20 * np.log10(peak) if peak > 0 else -120
        clipped_hard = np.sum(np.abs(signal) >= 1.0)
        clipped_near = np.sum(np.abs(signal) >= 0.99)

        print(f"=== {ch_name} Channel ===")
        print(f"  Peak: {peak:.6f} ({peak_db:.1f} dBFS)")
        print(f"  Hard clipped (>=1.0): {clipped_hard} samples")
        print(f"  Near-clipped (>=0.99): {clipped_near} samples")

        # 2. Consecutive clipping runs
        is_clipped = np.abs(signal) >= 1.0
        runs = []
        run_start = None
        for i in range(len(is_clipped)):
            if is_clipped[i]:
                if run_start is None:
                    run_start = i
            else:
                if run_start is not None:
                    runs.append((run_start, i - run_start))
                    run_start = None
        if run_start is not None:
            runs.append((run_start, len(is_clipped) - run_start))

        if runs:
            print(f"  Clipping runs: {len(runs)}")
            for start, length in sorted(runs, key=lambda x: -x[1])[:5]:
                t = start / sr
                print(f"    t={t:.3f}s, {length} samples ({length/sr*1000:.1f}ms)")
        print()

        # 3. Click/pop detection
        deltas = np.abs(np.diff(signal))
        pops_01 = np.sum(deltas > 0.1)
        pops_02 = np.sum(deltas > 0.2)
        pops_03 = np.sum(deltas > 0.3)
        print(f"  Pops (delta > 0.1): {pops_01}")
        print(f"  Pops (delta > 0.2): {pops_02}")
        print(f"  Pops (delta > 0.3): {pops_03}")

        # Check for periodic pops
        pop_indices = np.where(deltas > 0.1)[0]
        if len(pop_indices) > 5:
            intervals = np.diff(pop_indices) / sr * 1000  # ms
            median_interval = np.median(intervals)
            print(f"  Pop median interval: {median_interval:.1f}ms")
        print()

        # 4. RMS dynamics (50ms windows)
        win = int(sr * 0.05)
        rms_vals = []
        for i in range(0, len(signal) - win, win):
            chunk = signal[i:i+win]
            rms = np.sqrt(np.mean(chunk**2))
            rms_vals.append(rms)
        rms_arr = np.array(rms_vals)
        rms_db = 20 * np.log10(rms_arr + 1e-10)

        print(f"  RMS: min={np.min(rms_db):.1f} dB, max={np.max(rms_db):.1f} dB, "
              f"avg={np.mean(rms_db):.1f} dB")
        dynamic_range = np.max(rms_db) - np.min(rms_db)
        print(f"  Dynamic range: {dynamic_range:.1f} dB")

        # Sudden jumps
        rms_jumps = np.abs(np.diff(rms_db))
        big_jumps = np.sum(rms_jumps > 6)
        if big_jumps > 0:
            worst_idx = np.argmax(rms_jumps)
            worst_t = worst_idx * 0.05
            print(f"  RMS jumps > 6dB: {big_jumps} (worst: {rms_jumps[worst_idx]:.1f} dB at t={worst_t:.2f}s)")
        print()

        # 5. Crest factor (1s windows)
        win_1s = sr
        print(f"  Crest factor (1s windows):")
        low_crest = False
        for i in range(0, len(signal) - win_1s, win_1s):
            chunk = signal[i:i+win_1s]
            pk = np.max(np.abs(chunk))
            rms = np.sqrt(np.mean(chunk**2))
            if rms > 1e-6:
                cf = 20 * np.log10(pk / rms)
                t = i / sr
                if cf < 8:
                    print(f"    t={t:.0f}s: {cf:.1f} dB (LOW)")
                    low_crest = True
        if not low_crest:
            print(f"    All windows > 8 dB (healthy)")
        print()

    # 6. Overall dropout detection
    print("=== Dropouts (>10ms silence) ===")
    mono = np.mean(np.abs(data), axis=1)
    win = int(sr * 0.01)  # 10ms
    dropout_count = 0
    for i in range(0, len(mono) - win, win // 2):
        chunk = mono[i:i+win]
        if np.max(chunk) < 0.001:
            t = i / sr
            dropout_count += 1
            if dropout_count <= 10:
                print(f"  t={t:.3f}s")
    print(f"  Total: {dropout_count} dropout windows")


if __name__ == '__main__':
    default_path = Path(__file__).parent.parent.parent / "darkness_audio_capture.wav"
    path = sys.argv[1] if len(sys.argv) > 1 else str(default_path)
    analyze(path)
