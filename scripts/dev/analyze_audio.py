import wave
import struct
import math

wav = wave.open('/Users/jdv/code/darkness/darkness/darkness_audio_capture.wav', 'rb')
nchannels = wav.getnchannels()
sampwidth = wav.getsampwidth()
framerate = wav.getframerate()
nframes = wav.getnframes()
duration = nframes / framerate

print("Channels: {}, Sample width: {}, Rate: {}, Frames: {}, Duration: {:.2f}s".format(
    nchannels, sampwidth, framerate, nframes, duration))

# Read all frames
raw = wav.readframes(nframes)
wav.close()

# Parse samples (assuming 16-bit signed)
if sampwidth == 2:
    fmt = '<' + 'h' * (nframes * nchannels)
    samples = struct.unpack(fmt, raw)
elif sampwidth == 4:
    fmt = '<' + 'i' * (nframes * nchannels)
    samples = struct.unpack(fmt, raw)
else:
    print("Unexpected sample width: {}".format(sampwidth))
    exit(1)

# Focus on last 30 seconds
last_30s_start_frame = max(0, nframes - 30 * framerate)
last_30s_start_sample = last_30s_start_frame * nchannels
last_30s_samples = samples[last_30s_start_sample:]
last_30s_start_time = last_30s_start_frame / framerate

# Compute RMS in 200ms windows
window_frames = int(0.2 * framerate)
window_size = window_frames * nchannels
print("\nRMS analysis of last 30 seconds (starting at t={:.2f}s):".format(last_30s_start_time))
print("Window size: {} frames (200ms)".format(window_frames))
print("{:>10} {:>10} {:>8} {:>8} {:>20}".format("Time (s)", "RMS", "dB", "Peak", "Notes"))
print('-' * 60)

prev_rms = None
anomalies = []
window_data = []

for i in range(0, len(last_30s_samples) - window_size, window_size):
    chunk = last_30s_samples[i:i + window_size]
    # Mix to mono if stereo
    if nchannels == 2:
        mono = [(chunk[j] + chunk[j+1]) / 2 for j in range(0, len(chunk), 2)]
    else:
        mono = list(chunk)

    rms = math.sqrt(sum(s*s for s in mono) / len(mono))
    peak = max(abs(s) for s in mono)
    db = 20 * math.log10(rms / 32768.0) if rms > 0 else -100

    t = last_30s_start_time + (i / nchannels) / framerate

    notes = ''
    if rms < 10:
        notes = 'SILENCE/DROPOUT'
        anomalies.append((t, 'dropout', db))
    elif prev_rms is not None and prev_rms > 0:
        ratio = rms / prev_rms
        if ratio > 3.0:
            notes = 'JUMP UP {:.1f}x'.format(ratio)
            anomalies.append((t, 'jump up {:.1f}x'.format(ratio), db))
        elif ratio < 0.33:
            notes = 'DROP {:.2f}x'.format(ratio)
            anomalies.append((t, 'drop {:.2f}x'.format(ratio), db))

    window_data.append((t, rms, db, peak, notes))
    prev_rms = rms

# Print all windows
for t, rms, db, peak, notes in window_data:
    print("{:10.2f} {:10.1f} {:8.1f} {:8.0f} {:>20}".format(t, rms, db, peak, notes))

print("\n=== ANOMALIES DETECTED: {} ===".format(len(anomalies)))
for t, desc, db in anomalies:
    print("  t={:.2f}s: {} (level={:.1f} dB)".format(t, desc, db))

# Also analyze with finer granularity (50ms) around any anomalies
if anomalies:
    print("\n=== FINE-GRAINED ANALYSIS (50ms windows) around anomalies ===")
    fine_window_frames = int(0.05 * framerate)
    fine_window_size = fine_window_frames * nchannels

    for anom_t, anom_desc, _ in anomalies[:5]:  # first 5 anomalies
        # Look at +/- 2 seconds around anomaly
        anom_frame = int((anom_t - last_30s_start_time) * framerate)
        start = max(0, (anom_frame - 2 * framerate) * nchannels)
        end = min(len(last_30s_samples), (anom_frame + 2 * framerate) * nchannels)

        print("\n--- Around t={:.2f}s ({}) ---".format(anom_t, anom_desc))
        region = last_30s_samples[start:end]
        region_start_time = last_30s_start_time + start / nchannels / framerate

        for i in range(0, len(region) - fine_window_size, fine_window_size):
            chunk = region[i:i + fine_window_size]
            if nchannels == 2:
                mono = [(chunk[j] + chunk[j+1]) / 2 for j in range(0, len(chunk), 2)]
            else:
                mono = list(chunk)

            rms = math.sqrt(sum(s*s for s in mono) / len(mono)) if mono else 0
            db = 20 * math.log10(rms / 32768.0) if rms > 0 else -100
            t = region_start_time + (i / nchannels) / framerate
            marker = " <-- ANOMALY" if abs(t - anom_t) < 0.1 else ""
            print("  t={:.3f}s  RMS={:.1f}  dB={:.1f}{}".format(t, rms, db, marker))
