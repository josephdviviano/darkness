import csv
import math

rows = []
with open('/Users/jdv/code/darkness/darkness/darkness_audio_positions.csv') as f:
    reader = csv.reader(f)
    header = next(reader)
    print("Header:", header)
    for row in reader:
        ts = float(row[0])
        x, y, z = float(row[1]), float(row[2]), float(row[3])
        yaw, pitch = float(row[4]), float(row[5])
        voices = int(row[6])
        refl_voices = int(row[7])
        rows.append((ts, x, y, z, yaw, pitch, voices, refl_voices))

print("Total position samples:", len(rows))
print("Time range: {:.1f}ms to {:.1f}ms ({:.1f}s to {:.1f}s)".format(
    rows[0][0], rows[-1][0], rows[0][0]/1000, rows[-1][0]/1000))

# Find room transitions by looking for large position jumps or direction changes
# Also track speed
print("\n=== MOVEMENT ANALYSIS (speed, direction changes) ===")
print("{:>10} {:>8} {:>8} {:>8} {:>8} {:>6} {:>6} {:>10}".format(
    "Time(s)", "X", "Y", "Z", "Speed", "Vox", "Refl", "Notes"))

prev = None
speed_data = []
for r in rows:
    ts, x, y, z, yaw, pitch, voices, refl_voices = r
    t_sec = ts / 1000.0

    notes = ""
    speed = 0
    if prev is not None:
        dt = (ts - prev[0]) / 1000.0
        if dt > 0:
            dx = x - prev[1]
            dy = y - prev[2]
            dz = z - prev[3]
            speed = math.sqrt(dx*dx + dy*dy + dz*dz) / dt

            # Detect room changes (abrupt position or speed changes)
            if voices != prev[6]:
                notes += " VOX:{}->{}".format(prev[6], voices)
            if refl_voices != prev[7]:
                notes += " REFL:{}->{}".format(prev[7], refl_voices)
            if speed > 15:
                notes += " FAST"

    speed_data.append((t_sec, x, y, z, speed, voices, refl_voices, notes))
    prev = r

# Print data in the anomaly time ranges (recording is ~33s, positions are ~33s)
# Audio anomalies at: t=23.04s, 26.84s, 27.84s, 28.04s, 28.24s
# The timestamps may not align directly - let's print all data
print("\n=== FULL POSITION TRACE (every 5th sample) ===")
for i, (t, x, y, z, spd, vox, refl, notes) in enumerate(speed_data):
    if i % 5 == 0 or notes:
        print("{:10.2f} {:8.2f} {:8.2f} {:8.2f} {:8.1f} {:6d} {:6d}   {}".format(
            t, x, y, z, spd, vox, refl, notes))

# Now focus on the critical region - find where voice count or refl count changes
print("\n=== VOICE COUNT CHANGES ===")
prev_vox = None
prev_refl = None
for t, x, y, z, spd, vox, refl, notes in speed_data:
    if prev_vox is not None and (vox != prev_vox or refl != prev_refl):
        print("t={:.2f}s  voices: {}->{}  refl: {}->{}  pos=({:.1f},{:.1f},{:.1f})  speed={:.1f}".format(
            t, prev_vox, vox, prev_refl, refl, x, y, z, spd))
    prev_vox = vox
    prev_refl = refl

# Find when position Y crosses thresholds (hallway entry/exit)
print("\n=== Y-POSITION PROGRESSION (potential hallway traversal) ===")
print("Looking for Y-axis changes that indicate hallway transition...")
y_vals = [(t, y) for t, x, y, z, spd, vox, refl, notes in speed_data]
# Sample every 20th
for i in range(0, len(y_vals), 20):
    t, y = y_vals[i]
    sd = speed_data[i]
    print("t={:.2f}s  Y={:.2f}  X={:.2f}  Z={:.2f}  voices={} refl={}".format(
        t, y, sd[1], sd[3], sd[5], sd[6]))

# Find sustained high speed periods (walking through hallway)
print("\n=== SUSTAINED MOVEMENT PERIODS ===")
moving = False
move_start = None
for t, x, y, z, spd, vox, refl, notes in speed_data:
    if spd > 3.0 and not moving:
        moving = True
        move_start = t
    elif spd < 1.0 and moving:
        moving = False
        print("Movement: {:.2f}s to {:.2f}s ({:.1f}s duration)".format(
            move_start, t, t - move_start))
