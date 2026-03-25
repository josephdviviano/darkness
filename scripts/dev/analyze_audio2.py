import struct
import math

# Manual WAV parser for IEEE Float format
with open('/Users/jdv/code/darkness/darkness/darkness_audio_capture.wav', 'rb') as f:
    # RIFF header
    riff = f.read(4)
    assert riff == b'RIFF', "Not RIFF"
    file_size = struct.unpack('<I', f.read(4))[0]
    wave = f.read(4)
    assert wave == b'WAVE', "Not WAVE"

    fmt_found = False
    data_found = False
    nchannels = 0
    framerate = 0
    sampwidth = 0
    audio_format = 0

    while True:
        chunk_id = f.read(4)
        if len(chunk_id) < 4:
            break
        chunk_size = struct.unpack('<I', f.read(4))[0]

        if chunk_id == b'fmt ':
            fmt_data = f.read(chunk_size)
            audio_format = struct.unpack('<H', fmt_data[0:2])[0]
            nchannels = struct.unpack('<H', fmt_data[2:4])[0]
            framerate = struct.unpack('<I', fmt_data[4:8])[0]
            byte_rate = struct.unpack('<I', fmt_data[8:12])[0]
            block_align = struct.unpack('<H', fmt_data[12:14])[0]
            bits_per_sample = struct.unpack('<H', fmt_data[14:16])[0]
            sampwidth = bits_per_sample // 8
            fmt_found = True
            print("Format: {} (3=IEEE float), Channels: {}, Rate: {}, Bits: {}".format(
                audio_format, nchannels, framerate, bits_per_sample))
        elif chunk_id == b'data':
            # data_size might be 0 for streaming WAVs, read to end
            if chunk_size == 0:
                raw = f.read()
            else:
                raw = f.read(chunk_size)
            data_found = True
            break
        else:
            f.seek(chunk_size, 1)  # skip unknown chunk

    if not fmt_found or not data_found:
        print("Missing fmt or data chunk")
        exit(1)

    # Parse float samples
    if audio_format == 3 and sampwidth == 4:
        nsamples = len(raw) // 4
        samples = struct.unpack('<' + 'f' * nsamples, raw)
    else:
        print("Unsupported format: {} width: {}".format(audio_format, sampwidth))
        exit(1)

    nframes = nsamples // nchannels
    duration = nframes / framerate
    print("Total frames: {}, Duration: {:.2f}s".format(nframes, duration))

    # Focus on last 30 seconds
    last_30s_start_frame = max(0, nframes - 30 * framerate)
    last_30s_start_sample = last_30s_start_frame * nchannels
    last_30s_samples = samples[last_30s_start_sample:]
    last_30s_start_time = last_30s_start_frame / framerate

    # Compute RMS in 200ms windows (float samples are -1.0 to 1.0)
    window_frames = int(0.2 * framerate)
    window_size = window_frames * nchannels
    print("\nRMS analysis of last 30 seconds (starting at t={:.2f}s):".format(last_30s_start_time))
    print("{:>10} {:>10} {:>8} {:>8}   {}".format("Time(s)", "RMS", "dB", "Peak", "Notes"))
    print('-' * 70)

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

        rms = math.sqrt(sum(s*s for s in mono) / len(mono)) if mono else 0
        peak = max(abs(s) for s in mono) if mono else 0
        db = 20 * math.log10(rms) if rms > 1e-10 else -100

        t = last_30s_start_time + (i / nchannels) / framerate

        notes = ''
        if rms < 1e-6:
            notes = 'SILENCE/DROPOUT'
            anomalies.append((t, 'dropout', db))
        elif prev_rms is not None and prev_rms > 1e-8:
            ratio = rms / prev_rms
            if ratio > 3.0:
                notes = 'JUMP UP {:.1f}x'.format(ratio)
                anomalies.append((t, 'jump up {:.1f}x'.format(ratio), db))
            elif ratio < 0.33:
                notes = 'DROP {:.2f}x'.format(ratio)
                anomalies.append((t, 'drop {:.2f}x'.format(ratio), db))
            elif ratio > 2.0:
                notes = 'rise {:.1f}x'.format(ratio)
            elif ratio < 0.5:
                notes = 'fall {:.2f}x'.format(ratio)

        window_data.append((t, rms, db, peak, notes))
        prev_rms = rms

    # Print all windows
    for t, rms, db, peak, notes in window_data:
        print("{:10.2f} {:10.6f} {:8.1f} {:8.5f}   {}".format(t, rms, db, peak, notes))

    print("\n=== ANOMALIES DETECTED: {} ===".format(len(anomalies)))
    for t, desc, db in anomalies:
        print("  t={:.2f}s: {} (level={:.1f} dB)".format(t, desc, db))

    # Fine-grained analysis around anomalies (50ms windows)
    if anomalies:
        print("\n=== FINE-GRAINED (50ms) around anomalies ===")
        fine_window_frames = int(0.05 * framerate)
        fine_window_size = fine_window_frames * nchannels

        for anom_t, anom_desc, _ in anomalies[:5]:
            anom_sample_offset = int((anom_t - last_30s_start_time) * framerate) * nchannels
            start = max(0, anom_sample_offset - 2 * framerate * nchannels)
            end = min(len(last_30s_samples), anom_sample_offset + 2 * framerate * nchannels)

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
                db = 20 * math.log10(rms) if rms > 1e-10 else -100
                t = region_start_time + (i / nchannels) / framerate
                marker = " <-- ANOMALY" if abs(t - anom_t) < 0.15 else ""
                print("  t={:.3f}s  RMS={:.6f}  dB={:.1f}{}".format(t, rms, db, marker))
