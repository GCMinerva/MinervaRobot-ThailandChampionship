# Turret Calibration Workflow

## Yes, This Is Excellent for Calibration! ✅

You're absolutely right. Measuring actual encoder ticks per 360° rotation is the **gold standard** for motor calibration. This gives you:

✅ **Exact motor specs** - Every motor is slightly different
✅ **No guessing** - Real measurements, not estimates  
✅ **Precise autonomous** - Turret hits exact angles
✅ **Debugging tool** - Visual feedback on motor behavior

---

## Quick Start Workflow

### 1. **Run Calibration OpMode** (TurretCalibration.java)
   - Deploy code
   - Select `Turret Calibration` from OpMode list
   - Press START

### 2. **Perform Calibration Test** (Choose One)

#### **Option A: Manual Method (Recommended)**
```
1. Press A → Reset encoder to 0
2. Use Right Stick Y to rotate turret
3. Rotate SLOWLY and carefully 360° back to starting point
4. Press X to calculate ticks/degree
5. Note the value shown
```

**Why manual is better:**
- You control the rotation precisely
- Ensures you return to exact starting position
- More accurate for precise applications

#### **Option B: Auto Method (Quick Check)**
```
1. Press A → Reset encoder
2. Press Y → Auto rotates at 0.5 power for ~5 seconds
3. Results show estimated ticks/degree
```

**Why auto is useful:**
- Quick sanity check
- Doesn't require manual rotation
- Good for verifying manual results

---

## 3. Record Your Measurement

After calibration, you'll see something like:

```
Current Ticks: 10,240
Ticks per Degree: 28.44
```

---

## 4. Update TurretSubsystem

Open `TurretSubsystem.java` and find:

```java
public static final double TICKS_PER_DEGREE = 28.44;  // <-- CHANGE THIS
```

Replace `28.44` with your measured value. Example:

```java
public static final double TICKS_PER_DEGREE = 28.44;  // Measured on Nov 22
```

---

## 5. Verify It Works

Run `TurretTeleOpTest`:
- Turret should snap smoothly to target angles
- No overshooting or oscillating
- Should align within 1-2 degrees

If still not perfect, adjust PID gains:

```java
// In TurretSubsystem.java
private double kP = 0.01;  // Try 0.015 if undershooting, 0.008 if overshooting
private double kD = 0.001; // Try 0.002 if oscillating
```

---

## Files You Need

| File | Purpose |
|------|---------|
| `TurretCalibration.java` | **Calibration tool** - Run this first |
| `TurretSubsystem.java` | Update `TICKS_PER_DEGREE` with your measurement |
| `TurretTeleOpTest.java` | Verify it works after calibration |
| `AutoTurretRB3Test.java` | Your autonomous with turret tracking |
| `TURRET_CALIBRATION_GUIDE.md` | Detailed reference guide |

---

## Expected Results by Motor Type

These are **approximations** - your measurement may vary:

| Motor | RPM | Ticks/360° | Ticks/Degree |
|-------|-----|-----------|--------------|
| GoBilda 223 | 223 | ~10,240 | 28.44 |
| GoBilda 435 | 435 | ~8,000 | 22.22 |
| REV HD Hex | 312 | ~2,240 | 6.22 |
| TETRIX | 120 | ~1,440 | 4.00 |

**Your actual value may differ by ±10% depending on:
- Motor wear
- Gear tolerances  
- Mechanical friction
- Encoder calibration

That's why measuring is better than guessing!**

---

## Why This Matters

Without proper calibration:
- ❌ Turret overshoots target angle
- ❌ Autonomous shots miss
- ❌ Turret oscillates
- ❌ Takes forever to align

With proper calibration:
- ✅ Snaps to exact angles
- ✅ Reliable autonomous
- ✅ Smooth control
- ✅ Fast alignment

---

## Troubleshooting During Calibration

| Problem | Solution |
|---------|----------|
| Motor doesn't move | Check power cable, verify hardware name is "turret" |
| Encoder shows 0 | Check encoder cable connection |
| Can't rotate 360° | Turret has limits, check RobotHardwareConfig or mechanically |
| Huge tick numbers | Using wrong mode, ensure RUN_WITHOUT_ENCODER |
| Very small numbers | Motor may not have encoder, verify hardware |

---

## Pro Tips

1. **Rotate slowly** - More accurate measurement
2. **Mark starting position** - Use tape or mark on field
3. **Multiple tests** - Average 2-3 measurements for accuracy
4. **Watch the motor** - Look for smooth, consistent rotation
5. **Note external factors** - Battery voltage, friction, load all affect speed

Once you have your value, **write it down somewhere**. You'll need it if you rebuild the turret or swap motors!
