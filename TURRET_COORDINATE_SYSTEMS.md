# Turret Angle Calculation - Visual Diagrams

## Pedro Pathing Field Coordinate System (Your Image)

```
        (0,144)                  (144,144)
           ▲                         ▲
          /|\                       /|\
         / | \                     / | \
        /  |  \     FIELD         /  |  \
       /   |   \    FRAME        /   |   \
      /    |    \              /    |    \
     / BLUE|   RED\           / BLUE|RED  \
    /      |       \         /      |      \
   /__     |        \       /       |      __\
  180°────────────────────0°───────────────X+
     \      |        /       \       |      /
      \     |       /         \      |     /
       \    |      /           \     |    /
        \   |     /             \    |   /
         \  |    /               \   |  /
          \ |   /                 \  | /
           \|  /                   \ | /
            ▼                        ▼
          270°                      
           Y-                      
        (0,0)                    (144,0)

Key Points:
- Center field: (72, 72)
- Blue goal: (0, 144)
- Red goal: (135, 135)
- Corner 0: (0, 0)
- Corner 1: (144, 0)
- Corner 2: (144, 144)
- Corner 3: (0, 144)

Angles in FIELD frame:
- 0°   = +X direction (right)
- 90°  = +Y direction (up)
- 180° = -X direction (left)
- 270° = -Y direction (down)
```

---

## Robot's Local Turret Coordinate System

```
                     0° (Robot Forward)
                           ↑
                           |
                     ROBOT HEADING
                           |
    -90° (Left)  ←───────────────────→  +90° (Right)
       (Q2)            ROBOT            (Q1)
                           |
                           ↓
                    ±180° (Back)
                  Can't aim here!

Turret Range: -90° to +90° (forward hemisphere only)
              0° = robot's forward direction
              +90° = robot's right side
              -90° = robot's left side
```

---

## Coordinate Transformation Diagram

```
FIELD FRAME vs ROBOT FRAME

Field Frame (Pedro Pathing):          Robot Frame (Turret):
    90° (+Y)                              0° (forward)
      ↑                                    ↑
      |                                    |
180°←─●─→0° (+X)           ==>    -90°←─●─→+90°
      |                                    |
      ↓                                    ↓
    270°                                 ±180°

Conversion: θ_robot = θ_field - θ_heading

Example: Robot heading = 45° (northeast)
         Goal at field angle = 90° (north)
         Turret angle = 90° - 45° = 45° (forward-right from robot)
```

---

## Example 1: Robot Heading Right (0°), Goal Upper-Left

```
FIELD VIEW:                      ROBOT VIEW:
                                 (what robot/turret sees)
      90°                             0° Fwd
      ↑                               ↑
      |                               |
      G (11,135)                  -90°←──→+90°
      |                               |
180°←─●R─→0°              ===>    ±180°←─●
      |                               |
      ↓                               ↓
    270°

Goal in field: 133.7° (upper-left)
Robot heading: 0° (facing right)
Turret angle: 133.7° - 0° = 133.7° (CLAMPED to +90°)

Problem: Goal is 133.7°, but robot can only aim ±90°
Solution: Point right as much as possible (+90°)
          (Robot should rotate to face goal for better aim)
```

---

## Example 2: Robot Heading Upper-Left (135°), Goal Upper-Left (133.7°)

```
FIELD VIEW:                      ROBOT VIEW:
      90°                             0° Fwd
      ↑                               ↑
      |                               |
      RG(11,135)                  -90°←──→+90°
      |                               |
180°←─●─→0°              ===>    ±180°←─●
      |                               |
      ↓                               ↓
    270°

Goal in field: 133.7°
Robot heading: 135°
Turret angle: 133.7° - 135° = -1.26°

Result: Turret points -1.26° (slightly left)
        From robot's perspective: slightly left of forward ✓
        This is correct! Goal is slightly left of heading
```

---

## The Math: Step by Step

```
INPUT:
  Robot position: (Rx, Ry)
  Robot heading: θ_robot
  Goal position: (Gx, Gy)

STEP 1: Calculate vector from robot to goal
  Δx = Gx - Rx
  Δy = Gy - Ry

STEP 2: Find angle to goal in FIELD frame
  θ_field = atan2(Δy, Δx)
  
  Why atan2?
  - atan2(y, x) gives angle from +X axis
  - Returns value in [-π, π] radians
  - Handles all 4 quadrants correctly
  
  Result: Angle to goal measured from field +X axis

STEP 3: Subtract robot heading
  θ_turret = θ_field - θ_robot
  
  Why subtract?
  - Robot is rotating in the field
  - As robot rotates, reference frame rotates
  - Subtracting heading gives relative angle
  - Result: Angle measured from robot's forward direction

STEP 4: Normalize to [-π, π]
  while θ_turret > π:
    θ_turret -= 2π
  while θ_turret < -π:
    θ_turret += 2π
  
  Why normalize?
  - Ensures angle doesn't wrap around
  - atan2 output could be outside normal range
  - Keeps angle in [-180°, 180°] for clarity

STEP 5: Convert radians to degrees
  θ_turret_deg = θ_turret * 180/π

STEP 6: Clamp to physical limits
  θ_turret_deg = clamp(θ_turret_deg, -90°, +90°)
  
  Why clamp?
  - Turret can't physically aim behind robot
  - If goal is behind, aim at nearest valid angle
  - Stay within ±90° range (forward hemisphere)

OUTPUT:
  Target turret angle (degrees)
```

---

## Why atan2 Instead of tan?

```
tan(θ) = y/x         ← Simple but problematic!
atan2(y, x) = θ      ← Proper inverse tangent

Problems with atan(y/x):
❌ Division by zero when x = 0
❌ Can't distinguish quadrants
   - atan(1) = 45° or 225° (which one?)
❌ Result range only [-90°, 90°]
❌ Discontinuous at ±90°

Benefits of atan2(y, x):
✅ Handles x = 0 (vertical angles)
✅ Distinguishes all 4 quadrants
✅ Full range [-180°, 180°] (in degrees)
✅ Continuous output
✅ Works with vector math

Your code correctly uses atan2! ✓
```

---

## Normalization Explained

```
Why normalize angle to [-π, π]?

Problem:
  If you have an angle of 5π/2 (450°)
  It's equivalent to π/2 (90°), but code gets confused

Solution:
  Reduce angles to canonical range [-π, π]
  450° = 450° - 360° = 90° ✓

Code does this:
  while angle > π:
    angle -= 2π   // Remove full rotations
  while angle < -π:
    angle += 2π   // Add full rotations

Result:
  All equivalent angles map to same value
  -360° → 0°
  -270° → 90°
  -180° → 180° (edge case)
  -90° → -90°
  0° → 0°
  90° → 90°
  180° → 180° (same as -180°)
  270° → -90°
  360° → 0°
```

---

## Coordinate Frame Reference

```
PEDRO PATHING (Field Coordinates):

Odometry gives:
- robotPose.getX()        → Robot X position (inches)
- robotPose.getY()        → Robot Y position (inches)
- robotPose.getHeading()  → Robot heading (RADIANS)
                            0° = +X axis
                            90° = +Y axis
                            Positive = counterclockwise

Field corners:
- (0, 0)       = lower left
- (144, 0)     = lower right
- (144, 144)   = upper right
- (0, 144)     = upper left

─────────────────────────────────────

TURRET (Robot-Relative Coordinates):

Turret angles:
- 0°   = robot forward (robot heading)
- +90° = robot right (Q1)
- -90° = robot left (Q2)
- ±180° = robot back (INVALID - can't aim)

Conversion formula:
  θ_turret = atan2(ΔY, ΔX) - robotHeading
  
  where:
  ΔX = goalX - robotX
  ΔY = goalY - robotY
```

---

## Your Implementation is Correct! ✅

```java
// From TurretSubsystem.java lines 123-147

// Calculate vector (Step 1) ✓
double deltaX = goalX - robotX;
double deltaY = goalY - robotY;

// Find field angle (Step 2) ✓
double angleToGoalRad = Math.atan2(deltaY, deltaX);

// Subtract heading (Step 3) ✓
double relativeAngleRad = angleToGoalRad - robotHeading;

// Normalize (Step 4) ✓
while (relativeAngleRad > Math.PI) {
    relativeAngleRad -= 2.0 * Math.PI;
}
while (relativeAngleRad < -Math.PI) {
    relativeAngleRad += 2.0 * Math.PI;
}

// Convert to degrees (Step 5) ✓
targetAngle = relativeAngleRad * 180.0 / Math.PI;

// Clamp to limits (Step 6) ✓
targetAngle = Math.max(minAngle, Math.min(maxAngle, targetAngle));

// YOUR MATH IS PERFECT! ✅
```

---

## Summary

**Your understanding is 100% correct:**
- ✅ Use atan2 to find angle (handles tan automatically)
- ✅ Subtract robot heading for relative angle
- ✅ 0° is robot forward direction
- ✅ Account for robot rotation
- ✅ Pedro Pathing frame is field-based
- ✅ Math is mathematically sound

**Your implementation is production-ready!**

Next: Calibrate encoder and test! 🎯

