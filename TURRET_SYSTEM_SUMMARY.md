# Turret System Summary

## What You Have Now

You now have a complete **turret aiming system** with encoder-based closed-loop control for your Decode FTC robot.

---

## Files Created

### **Subsystems**
- `TurretSubsystem.java` - Main turret control with encoder feedback and PID
  - Calculates angle to goal automatically
  - Closes the loop using encoder feedback
  - Provides alignment checking
  - **Action**: Update `TICKS_PER_DEGREE` after calibration

### **Autonomous**
- `AutoTurretRB3Test.java` - Your existing auto with turret tracking integrated
  - Calls `turret.update(robotPose)` in every loop
  - Turret continuously aims at goal while robot follows paths
  - Shows turret angle and alignment on telemetry

### **Calibration Tools**
- `TurretCalibration.java` - **Calibration OpMode** (RUN THIS FIRST)
  - Lets you manually rotate turret and measure encoder ticks
  - Calculates ticks per degree
  - Two modes: manual rotation or auto 360° test
  
### **Testing**
- `TurretTeleOpTest.java` - Test OpMode to verify turret works
  - Blue goal fixed (11, 135)
  - Right stick controls turret angle
  - Shows encoder position and alignment status

### **Documentation**
- `TURRET_CALIBRATION_GUIDE.md` - Detailed calibration reference
- `TURRET_CALIBRATION_WORKFLOW.md` - Quick start workflow

---

## Recommended Next Steps

### **Step 1: Calibrate (20 minutes)**
```bash
1. Deploy TurretCalibration.java
2. Run on robot
3. Press A to reset encoder
4. Rotate turret 360° manually (or press Y for auto)
5. Note the ticks/degree value
6. Update TurretSubsystem.TICKS_PER_DEGREE
```

### **Step 2: Test (10 minutes)**
```bash
1. Deploy TurretTeleOpTest.java
2. Run on robot
3. Use right stick to move turret
4. Verify it snaps smoothly to angles
5. Check alignment is within 1-2°
```

### **Step 3: Run Autonomous (5 minutes)**
```bash
1. Deploy AutoTurretRB3Test.java
2. Set robot at starting position
3. Press START
4. Watch turret track goal while following paths
5. Verify "Aligned" shows true near goal
```

### **Step 4: Fine-tune PID (if needed)**
```java
// If turret overshoots/undershoots/oscillates:
turret.setPIDGains(kP, kI, kD);
// kP: increase if too slow, decrease if too fast
// kD: increase if oscillates
```

---

## Hardware Setup Checklist

- [ ] Turret motor connected and named `"turret"` in hardware console
- [ ] Motor has encoder connected (should see changing ticks)
- [ ] Motor power is adequate (recommend 12V battery)
- [ ] Turret can rotate freely within ±90° range
- [ ] No mechanical binds or friction issues
- [ ] Motor direction is correct (use negative power to move in expected direction)

---

## Coordinate System Reference

```
                     Robot Forward (0°)
                            |
                            |
      Left Side (-90°) <-----+-----> Right Side (+90°)
                    (Q2)            (Q1)
                            |
                         Goal
                            
- 0° = Robot's forward direction
- +90° = Right side (Q1)
- -90° = Left side (Q2)
- Range: ±90° from robot heading
```

The turret automatically calculates the angle from the robot's current position to the goal, so as the robot moves and rotates, the turret adjusts accordingly.

---

## Key Methods

```java
// In your autonomous or TeleOp:

// Set goal position (field coordinates in inches)
turret.setGoalPosition(11.0, 135.0);  // Blue goal

// Update turret toward goal (call in loop)
boolean aligned = turret.update(robotPose);  // Returns true if aligned

// Check if ready to shoot
if (turret.isAligned()) {
    // Shoot!
}

// Manual angle control (for testing)
turret.setGoalPosition(x + 100*cos(angle), y + 100*sin(angle));

// Adjust PID gains (for tuning)
turret.setPIDGains(kP, kI, kD);

// Reset to center
turret.resetToCenter();

// Stop motor
turret.stop();
```

---

## Telemetry Output

When running, you'll see:
```
=== ROBOT STATE ===
Robot X: 72.0 in
Robot Y: 72.0 in  
Robot Heading: 45.0°

=== TURRET STATE ===
Current Angle: -15.3°
Target Angle: -14.8°
Angle Error: -0.5°
Encoder Ticks: -425
Aligned: true

=== GOAL (FIXED) ===
Goal X: 11.0 in
Goal Y: 135.0 in
```

This shows exactly where the turret is and where it needs to be.

---

## Expected Performance

After calibration:

| Metric | Expected |
|--------|----------|
| Alignment time | < 500ms |
| Alignment accuracy | ±1-2° |
| Movement smoothness | Smooth, no oscillation |
| Encoder feedback | Visible ticks updating |
| Autonomous tracking | Follows goal as robot moves |

---

## Common Issues & Fixes

| Issue | Fix |
|-------|-----|
| Turret doesn't move | Check motor name, power, connections |
| Oscillates | Decrease kD or increase alignmentTolerance |
| Undershoots | Increase kP or TICKS_PER_DEGREE |
| Overshoots | Decrease kP or increase kD |
| Slow response | Increase kP or maxPower |
| Encoder shows 0 | Check encoder cable connection |

---

## Questions?

- **Can I shoot while moving?** Yes! The `update()` call continuously adjusts as the robot moves.
- **Can I change goal mid-auto?** Yes! Just call `turret.setGoalPosition(newX, newY)`.
- **What if turret hits limit?** It clamps to ±90° automatically.
- **Can I use different motor?** Yes! Calibrate for your specific motor's ticks/degree.
- **How fast does it update?** As fast as your robot's main loop (usually 60-100 Hz).

---

## Next: Red Goal Variant

When ready, create `AutoTurretRB3TestRed.java` that:
```java
turret.setGoalPosition(135.0, 135.0);  // Red goal
```

Then you'll have autonomous for both alliances!
