# Turret Encoder Calibration Guide

## Why Calibrate?

The encoder value `TICKS_PER_DEGREE` is critical for precision. Different motors have different gear ratios:

- **GoBilda 223 RPM**: ~10,240 ticks/revolution → **~28.44 ticks/degree**
- **GoBilda 435 RPM**: ~8,000 ticks/revolution → **~22.22 ticks/degree**
- **Other motors**: Varies by gearbox

Using the wrong value = turret moves to wrong angles → miss shots.

---

## Calibration Process (Manual Method)

### Step 1: Run TurretCalibration OpMode
1. Deploy to robot
2. Select `Turret Calibration` from OpMode menu
3. Press **START**

### Step 2: Reset Encoder
- Press **A** button
- Telemetry should show "Encoder reset to 0"

### Step 3: Rotate Turret 360°
- Use **Right Stick Y** to apply power
- Rotate the turret **slowly and carefully**
- Rotate exactly **360° back to starting position** (full rotation)
- **Important**: Return to exact starting position for accurate measurement

### Step 4: Read Encoder Ticks
- Note the number shown under "Current Ticks"
- This is total ticks for 360°

### Step 5: Calculate Ticks Per Degree
```
Ticks Per Degree = Total Ticks / 360

Example:
- If encoder shows 10,240 ticks after 360° rotation
- Ticks Per Degree = 10,240 / 360 = 28.44
```

### Step 6: Update TurretSubsystem
Edit `TurretSubsystem.java`:

```java
private static final double TICKS_PER_REVOLUTION = 10240.0;  // Update this value
private static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;  // Auto-calculated
```

Or directly:
```java
private static final double TICKS_PER_DEGREE = 28.44;  // Your measured value
```

---

## Calibration Process (Automatic Method)

If manual rotation is difficult:

1. Press **A** to reset encoder
2. Press **Y** button for auto 360° rotation test
3. OpMode will rotate motor at 0.5 power for ~5 seconds
4. Results show calculated `TICKS_PER_DEGREE`
5. Update TurretSubsystem with the value

**Note**: Auto method is less precise. Manual is better if possible.

---

## Troubleshooting

### Motor doesn't move
- Check hardware name is `"turret"` in hardware console
- Check motor power cable connection
- Verify power supply is on

### Encoder shows 0 ticks
- Encoder may not be connected
- Check encoder cable on motor
- Verify motor type supports encoder

### Numbers way too high/low
- Wrong motor type in code (RUN_WITHOUT_ENCODER mode should not use encoder)
- Measure again carefully
- Ensure turret is exactly at starting position when reading

### Turret overshoots in autonomous
- TICKS_PER_DEGREE is too low
- Measure again more carefully
- Increase PID gains (kP) slightly

### Turret undershoots in autonomous
- TICKS_PER_DEGREE is too high
- Decrease PID gains (kP) slightly

---

## Quick Reference

| Motor Type | Expected Ticks/360° | Ticks/Degree |
|------------|-------------------|--------------|
| GoBilda 223 RPM | ~10,240 | 28.44 |
| GoBilda 435 RPM | ~8,000 | 22.22 |
| TETRIX | ~1,440 | 4.00 |
| REV HD Hex | ~2,240 | 6.22 |

If your measurement differs significantly from these, double-check your motor type!

---

## Testing After Calibration

After updating `TICKS_PER_DEGREE`:

1. Run `TurretTeleOpTest`
2. Verify turret snaps to target angles
3. Check that 0° points directly forward
4. Test ±90° limits work correctly
5. Verify alignment tolerance is accurate

If still not working:
- Adjust PID gains in `TurretSubsystem.setPIDGains()`
- Increase `alignmentTolerance` if too strict
