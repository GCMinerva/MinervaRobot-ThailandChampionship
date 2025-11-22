# Turret Angle Calculation - Mathematical Breakdown

## Your Field Coordinate System (From Image)

```
Field View (Pedro Pathing Reference Frame):
                    90°
                    ↑ (+Y axis)
                    |
    180° ← ─────────┼───────── → 0° (+X axis)
                    |
                    ↓ 270°
                  (-Y axis)

Example Points:
- Blue Goal: (0, 144)      = Upper left corner
- Red Goal: (135, 135)     = Upper right  
- Robot Start: (72, 72)    = Center
- Corner (0, 0)            = Lower left
- Corner (144, 0)          = Lower right
- Corner (144, 144)        = Upper right
- Corner (0, 144)          = Upper left
```

## Robot's Local Coordinate System

```
Robot Local Frame (Turret Reference):
                    0° (robot forward)
                         |
                         ↑
    -90° (left) ← ───────●───────  → +90° (right)
                  (Q2)   ROBOT   (Q1)
                         ↓
                    ±180° (back)
```

## The Math You Described ✅

### Step 1: Calculate Vector from Robot to Goal
```
Δx = goalX - robotX
Δy = goalY - robotY

Example:
- Robot at (72, 72), Goal at (0, 144)
- Δx = 11 - 72 = -61 (left of robot)
- Δy = 135 - 72 = +63 (above robot)
```

### Step 2: Find Angle in Field Coordinates
```
θ_field = atan2(Δy, Δx)

Using atan2 because it handles all 4 quadrants:
θ_field = atan2(63, -61)
θ_field ≈ 133.4° (in field frame)

This angle is in FIELD coordinates (0° = +X axis, CCW)
```

### Step 3: Account for Robot's Heading
```
Robot heading from Pedro: θ_robot (in radians)

The turret angle relative to robot is:
θ_turret = θ_field - θ_robot

Normalize to [-π, π]:
if θ_turret > π:
    θ_turret -= 2π
if θ_turret < -π:
    θ_turret += 2π
    
Convert to degrees: θ_turret_deg = θ_turret * 180/π
```

### Step 4: Verify Result
```
Example:
- θ_field = 133.4° = 2.33 rad
- Robot heading = 0° = 0 rad
- θ_turret = 2.33 - 0 = 2.33 rad = 133.4°

Meaning: Turret needs to aim at 133.4° (upper left in field frame)
Since robot heading is 0° (pointing right):
  - 90° from field = upper direction = +90° to turret (right from robot view)
  - No wait, let me recalculate...

Actually with your field orientation:
- Robot at center facing right (0° heading)
- Goal at upper-left (133.4° in field frame)
- From robot's perspective (looking right):
  - Upper-left would be... behind and left
  - That's approximately 133.4° - 0° = 133.4° in field frame
  - But we need to convert to robot-relative (-90° to +90°)

If angle > 90°, we're looking backward/left
Convert: θ_turret = θ_field - θ_robot (this gives field-relative angle)
Then clamp to ±90° range if needed

But wait, atan2 can go past ±90°...
```

## The Issue: Your ±90° Constraint

You said turret can only rotate ±90° (Q1 and Q2, not behind robot):

```
This means:
- If goal is to the right: 0° to +90° ✓
- If goal is to the left: -90° to 0° ✓
- If goal is behind: Can't aim there ✗

So we need to handle cases where atan2 gives us an angle outside ±90°
relative to robot heading.

Example: If goal is behind robot
- θ_field = 180°
- θ_robot = 0° (robot facing +X)
- θ_turret = 180° - 0° = 180° (behind, outside ±90°)
- This is INVALID for your turret!
```

---

## Corrected Formula

```javascript
// 1. Calculate vector to goal
Δx = goalX - robotX
Δy = goalY - robotY

// 2. Angle to goal in FIELD frame (atan2 returns radians)
θ_field_rad = atan2(Δy, Δx)

// 3. Robot's heading in field frame (also in radians from Pedro)
θ_robot_rad = robotHeading  // Already in radians from Pedro

// 4. Calculate turret angle relative to robot
θ_turret_rad = θ_field_rad - θ_robot_rad

// 5. Normalize to [-π, π] range
while (θ_turret_rad > π):
    θ_turret_rad -= 2π
while (θ_turret_rad < -π):
    θ_turret_rad += 2π

// 6. Convert to degrees
θ_turret_deg = θ_turret_rad * 180/π

// 7. Check if goal is within ±90° range
if |θ_turret_deg| > 90:
    // Goal is behind robot, can't aim there!
    // Option 1: Clamp to nearest extreme (±90°)
    // Option 2: Don't update turret
    // Option 3: Rotate robot to face goal first

    θ_turret_deg = clamp(θ_turret_deg, -90, +90)
```

---

## Visual Example With Your Field

```
Scenario 1: Robot at (72,72) facing right (0°), Goal at (0,144)
─────────────────────────────────────────────────────────────

Field coordinates:
  Δx = 11 - 72 = -61
  Δy = 135 - 72 = +63
  
  θ_field = atan2(63, -61) = 2.33 rad = 133.4°
  
Robot heading: 0°

Turret angle: 133.4° - 0° = 133.4°

Problem: 133.4° is > 90°, so goal is partially behind robot!
Solution: Clamp to +90° (turret points to the right, can't reach goal)
          OR rotate robot to face goal first

─────────────────────────────────────────────────────────────

Scenario 2: Robot at (72,72) facing upper-left (135°), Goal at (0,144)
─────────────────────────────────────────────────────────────

Field coordinates:
  Δx = 11 - 72 = -61
  Δy = 135 - 72 = +63
  
  θ_field = atan2(63, -61) = 2.33 rad = 133.4°

Robot heading: 135° = 2.36 rad

Turret angle: 133.4° - 135° = -1.6° = -0.028 rad ≈ -1.6°

This is within [-90°, +90°] ✓
Meaning: Aim slightly left from robot forward
         Robot rotates to face goal, turret adjusts fine

─────────────────────────────────────────────────────────────

Scenario 3: Robot at (72,72) facing upper-left (135°), Goal at (0,144)
           Using proper radian math
─────────────────────────────────────────────────────────────

Δx = -61, Δy = +63
θ_field = atan2(63, -61) ≈ 2.3346 rad
θ_robot = 135° × π/180 = 2.3562 rad

θ_turret = 2.3346 - 2.3562 = -0.0216 rad
Convert: -0.0216 × 180/π ≈ -1.24°

Result: Turret points slightly left ✓
```

---

## Your Code Should Do

```java
double deltaX = goalX - robotX;
double deltaY = goalY - robotY;

// Angle to goal in field frame (radians)
double angleToGoalRad = Math.atan2(deltaY, deltaX);

// Robot heading in field frame (already in radians from Pedro)
double robotHeadingRad = robotPose.getHeading();

// Turret angle relative to robot (radians)
double turretAngleRad = angleToGoalRad - robotHeadingRad;

// Normalize to [-π, π]
while (turretAngleRad > Math.PI) {
    turretAngleRad -= 2.0 * Math.PI;
}
while (turretAngleRad < -Math.PI) {
    turretAngleRad += 2.0 * Math.PI;
}

// Convert to degrees
double turretAngleDeg = turretAngleRad * 180.0 / Math.PI;

// Clamp to ±90° range (turret physical limits)
turretAngleDeg = Math.max(-90.0, Math.min(90.0, turretAngleDeg));

// This is your target angle!
targetAngle = turretAngleDeg;
```

---

## Your Understanding: CORRECT! ✅

✅ **You understood:**
- Calculate Δx and Δy (vector to goal)
- Use atan2(Δy, Δx) to get angle to goal
- Subtract robot heading to get relative angle
- Result is turret angle from robot perspective

✅ **Key insight:**
- 0° turret = robot forward direction
- Robot heading changes → turret reference frame rotates
- Subtracting heading accounts for this automatically

✅ **Your math is sound!**

The code in TurretSubsystem already does this correctly. Verify it matches the logic above!

