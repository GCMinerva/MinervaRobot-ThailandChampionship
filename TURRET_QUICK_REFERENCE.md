# Turret System - Quick Reference Guide

## Your Calibration Workflow (What To Do Right Now)

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  STEP 1: RUN TurretCalibration.java OpMode             │
│  ─────────────────────────────────────────────────────  │
│  - Deploy and select "Turret Calibration"              │
│  - Press START on phone                                │
│  - Press A to reset encoder                            │
│  - Slowly rotate turret 360° with Right Stick          │
│  - Press X to calculate                                │
│  - 📝 WRITE DOWN: ticks_per_degree value               │
│                                                         │
│  Expected: ~28.44 ticks/degree (±10%)                 │
│                                                         │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  STEP 2: UPDATE TurretSubsystem.java                   │
│  ─────────────────────────────────────────────────────  │
│  public static final double TICKS_PER_DEGREE = 28.44;  │
│                                  ↑                      │
│                            YOUR VALUE HERE              │
│                                                         │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  STEP 3: RUN TurretTeleOpTest.java OpMode              │
│  ─────────────────────────────────────────────────────  │
│  - Deploy and select "Turret TeleOp Test"              │
│  - Press START                                         │
│  - Use Right Stick Y to move turret                    │
│  - Verify smooth movement to ±90°                      │
│  - Check "Aligned: true" shows on telemetry            │
│  - Press A to reset to center                          │
│                                                         │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  STEP 4: RUN AutoTurretRB3Test.java (Your Auto)        │
│  ─────────────────────────────────────────────────────  │
│  - Deploy your autonomous                              │
│  - Turret automatically tracks goal while running      │
│  - Aiming at Blue Goal (11, 135)                       │
│  - Watch telemetry: Turret Angle & Aligned status      │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Your Turret Angle Visualization

```
                          ROBOT
                            |
                       (0° forward)
                            |
        -45°               -90°               +45°
          \                 |                 /
           \                |                /
            \               |               /
             \              |              /
              \             |             /
               ╲            |            ╱  ← Range: ±90°
                ╲           |           ╱
                 ╲    GOAL  |    B     ╱     (Robot relative)
                  ╲         |  GOAL   ╱
                   ╲        |        ╱
                    ╲       |       ╱
                     ╲      |      ╱
                      ╲     |     ╱
                       ╲    |    ╱
                      -90°  |   +90°
                            
        Q2 (LEFT)          CENTER          Q1 (RIGHT)
        -90° to 0°         0°              0° to +90°
```

---

## File Organization

```
MinervaRobot/
├── TeamCode/
│   └── src/main/java/
│       └── org/firstinspires/ftc/teamcode/
│           ├── archived/
│           │   ├── subsystems/
│           │   │   └── TurretSubsystem.java ⭐ (MAIN SYSTEM)
│           │   │       └── Update TICKS_PER_DEGREE here
│           │   │
│           │   ├── auto/test/
│           │   │   └── AutoTurretRB3Test.java (YOUR AUTO with turret)
│           │   │
│           │   └── opmodes/
│           │       ├── TurretCalibration.java ⭐ (RUN FIRST)
│           │       ├── TurretTeleOpTest.java (TEST IT)
│           │       ├── TURRET_CALIBRATION_GUIDE.md
│           │       └── TURRET_CALIBRATION_WORKFLOW.md
│           │
│           └── RobotHardwareConfig.java (Has turret motor config)
│
├── TURRET_SYSTEM_SUMMARY.md (THIS OVERVIEW)
└── TURRET_CALIBRATION_WORKFLOW.md
```

---

## Control Mappings

### TurretCalibration.java (Calibration Mode)
```
Right Stick Y:  Apply power to motor (positive = forward)
Left Stick Y:   Fine power adjustment (±0.1 increments)
DPad Up/Down:   Quick power adjustment (±0.05)
A:              Reset encoder to 0 ✓ DO THIS FIRST
B:              Take snapshot of current ticks
X:              Calculate ticks/degree from snapshot
Y:              Auto 360° rotation test
```

### TurretTeleOpTest.java (Testing Mode)
```
Right Stick Y:  Control turret angle manually (-90° to +90°)
A:              Reset turret to center (0°)
```

### AutoTurretRB3Test.java (Autonomous)
```
Automatic!      Turret tracks blue goal as robot follows paths
                Shows alignment status on telemetry
```

---

## Encoder Ticks Reference

### What the Ticks Mean
```
Position (degrees) = Encoder Ticks / TICKS_PER_DEGREE

Example:
- TICKS_PER_DEGREE = 28.44
- Encoder reads: 2844 ticks
- Position = 2844 / 28.44 = 100°
```

### How Many Ticks = What Angle?
```
At 28.44 ticks/degree (GoBilda 223 RPM):

0°   =       0 ticks  (center/forward)
45°  =    1,280 ticks  (right side, halfway)
90°  =    2,560 ticks  (full right)
-45° =   -1,280 ticks  (left side, halfway)
-90° =   -2,560 ticks  (full left)
360° =   10,240 ticks  (full rotation)
```

---

## Expected Encoder Ranges by Motor

```
Motor Type          Ticks/360°      Ticks/Degree
─────────────────────────────────────────────────
GoBilda 223 RPM     10,240          28.44
GoBilda 435 RPM      8,000          22.22
REV HD Hex           2,240           6.22
TETRIX              1,440            4.00
Custom Motor        ❓ MEASURE IT!    ???
```

---

## Troubleshooting Flowchart

```
            Turret not moving?
                   |
        ┌──────────┴──────────┐
        ↓                     ↓
    Motor has    Motor spins but
    no power?    turret doesn't?
        |              |
        ↓              ↓
    Check:        Check:
    - Power cable  - Motor mounting
    - Battery      - Gearbox connection
    - Motor name   - Friction/binding
    in hardware
    console
        |              |
        └──────┬───────┘
               ↓
        Still not working?
               |
        Check encoder ticks:
        - If 0: encoder not connected
        - If increasing: motor OK, mechanical issue
        - If not increasing: encoder issue
```

---

## Success Criteria Checklist

- [ ] Motor spins smoothly with gamepad input
- [ ] Encoder ticks increase/decrease when motor moves
- [ ] TurretCalibration shows ~28 ticks/degree
- [ ] TurretTeleOpTest moves to target angles
- [ ] "Aligned: true" appears on telemetry when at target
- [ ] Autonomous turret tracks goal while following paths
- [ ] No oscillation or jerky movement
- [ ] Turret stays within ±90° limits
- [ ] Alignment tolerance is ±2° or better

---

## Performance Targets

| Goal | Target | How to Verify |
|------|--------|---------------|
| Calibration Accuracy | ±5% error | Repeat test 2-3 times |
| Alignment Time | < 500ms | Watch telemetry in TeleOp |
| Alignment Tolerance | ±2° | "Aligned: true" shows |
| Movement Smoothness | No oscillation | Visual inspection |
| Encoder Precision | ±1 tick | Check encoder on startup |
| Autonomous Tracking | Smooth aim | Watch during auto |

---

## Next Steps After Getting It Working

1. **Red Goal Variant**
   ```java
   // Create AutoTurretRB3TestRed.java
   turret.setGoalPosition(135.0, 135.0);  // Red goal
   ```

2. **Optimize PID Gains**
   ```java
   // Fine-tune for your robot
   turret.setPIDGains(0.015, 0.0, 0.002);  // Example
   ```

3. **Add Shooting Integration**
   ```java
   if (turret.isAligned()) {
       shooter.shoot();  // Only when turret ready
   }
   ```

4. **Test Multiple Positions**
   - Test from different starting positions
   - Test with different robot headings
   - Verify turret tracks correctly in all cases

---

## Pro Tips

💡 **Tip 1**: Write down your TICKS_PER_DEGREE value somewhere safe. You'll need it if you rebuild or swap motors.

💡 **Tip 2**: Rotate slowly during calibration for best accuracy. Rushing gives bad measurements.

💡 **Tip 3**: Check motor direction! Negative power should rotate counter-clockwise (when looking down).

💡 **Tip 4**: If turret oscillates, increase kD slightly. If too slow, increase kP.

💡 **Tip 5**: Test at competition temperature. Cold/hot batteries affect motor speed.

---

## Emergency Reset

If turret gets stuck:
```java
// In TurretSubsystem constructor
turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
```

Then manually rotate turret to center position and:
```java
turret.resetToCenter();
```

---

**You're all set! Run TurretCalibration.java first, then follow the steps. Your turret will be amazing! 🎯**
