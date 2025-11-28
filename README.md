# MinervaRobot - FTC Robot Controller

FTC (FIRST Tech Challenge) robot control system with advanced autonomous capabilities, command-based programming, and ultra-fast hot-reload deployment.

## 🚀 Key Features

### ⚡ SlothLoad - Lightning Fast Deployment
- **1-2 second code deployment** using Sloth hot-reload technology
- No app restarts required - code changes are applied instantly
- Dramatically faster development cycles compared to traditional 30-40 second deployments
- Safe to deploy while other code is running

### 🤖 Command-Based Architecture
- Built with **FTCLib** command-based programming framework
- Modular, reusable command structure for robot actions
- Clean separation of concerns between subsystems and commands
- Easy to test and maintain

### 🎯 Advanced Autonomous Features
- **Pedro Pathing** integration for precise path following
- Percentage-based action triggers along paths
- AprilTag vision detection and tracking
- Complex multi-path autonomous routines

### 🎮 Robot Subsystems
- **TurretSubsystem** - PID-controlled turret with automatic goal tracking (±90° range)
- **ActuatorSubsystem** - Dual shooter motors, intake system, and multi-position sorter
- **AprilTagSubsystem** - Real-time vision processing for alliance color detection
- **LiftingSubsystem** - Dual servo lifting mechanism for vertical specimen placement

### 📊 Real-Time Telemetry
- **SlothBoard Dashboard** integration for live robot monitoring
- Custom telemetry displays for debugging
- Path progress visualization
- Pose tracking (X, Y, Heading)

## 🛠️ Technology Stack

### Core Libraries
- **FTC SDK 11.0.0** - Official FIRST Tech Challenge SDK
- **FTCLib 2.1.1** - Command-based programming framework
- **Pedro Pathing 2.0.2** - Advanced path following library
- **Sloth 0.2.4** - Hot-reload deployment system
- **Dairy Core 2.2.4** - Sinister runtime for dynamic loading

### Vision & AI
- **EasyOpenCV 1.5.3** - Computer vision processing
- **TensorFlow Lite 2.16.1** - Neural network inference
- **TensorFlow Lite GPU** - Hardware-accelerated AI

### Development Tools
- **SlothBoard Dashboard** - Real-time robot monitoring
- **FullPanels 1.0.6** - Enhanced UI components

## 📁 Project Structure

```
MinervaRobot/
├── TeamCode/
│   └── src/main/java/org/firstinspires/ftc/teamcode/
│       ├── archived/
│       │   ├── auto/test/
│       │   │   ├── commands/          # Command-based commands
│       │   │   │   ├── OpModeCommand.java
│       │   │   │   ├── FollowPath.java
│       │   │   │   └── WaitUntilPathPercent.java
│       │   │   └── opmodes/           # Autonomous OpModes
│       │   │       └── AutoDawnCommand.java
│       │   └── subsystems/            # Robot subsystems
│       │       ├── ActuatorSubsystem.java
│       │       └── AprilTagSubsystem.java
│       ├── auto/                      # Autonomous routines
│       ├── pedroPathing/              # Pedro Pathing configuration
│       └── util/                      # Utility classes
└── FtcRobotController/                # FTC Robot Controller app
```

## 🤖 Robot Subsystems

### � TurretSubsystem
Precision turret control with encoder feedback for goal alignment.

**Features:**
- Closed-loop PID control for accurate positioning
- Automatic goal tracking using field coordinates
- ±90° rotation range (left/right from robot heading)
- Real-time alignment feedback

**Key Methods:**
- `setGoalPosition(goalX, goalY)` - Set target field coordinates
- `update(robotPose)` - Update turret angle based on robot position
- `isAligned()` - Check if turret is on target
- `setStartingAngle(degrees)` - Initialize turret position
- `configureControl(maxPower, minPower, tolerance, kP, kI, kD)` - Tune PID
- `resetToCenter()` - Return to 0° (robot forward)

**Use Case:**
```java
turret.setGoalPosition(144, 144); // Aim at goal
while (!turret.isAligned()) {
    turret.update(follower.getPose());
}
shooter.fire(); // Shoot when aligned
```

---

### 🔧 ActuatorSubsystem
Controls shooter wheels, intake, and game piece sorting mechanism.

**Components:**
- **Dual Shooter Motors** (outtake1, outtake2) - Variable speed shooting
- **Intake Motor** - Game piece collection
- **Bin Servos** (binit, brinit) - Ball feeding mechanism
- **Sorter Servo** - Multi-position sample sorting

**Shooter Methods:**
- `runShooterFar()` - High power for long-range shots
- `runShooterClose()` - Lower power for close shots
- `stopShooter()` - Stop shooter motors
- `justShoot()` - Quick shot sequence
- `FarGO()` - Far distance shot with bin control
- `MidGO()` - Medium distance shot
- `NearGO()` - Close distance shot

**Intake Methods:**
- `runIntake()` - Start intake motor
- `stopIntake()` - Stop intake motor

**Sorter Methods:**
- `setSorterP1()`, `setSorterP2()`, `setSorterP3()` - Preset positions for shooting
- `setSorterIN1()`, `setSorterIN2()`, `setSorterIN3()` - Positions for intake
- `setSorter(position)` - Custom position control

**Bin Control:**
- `openBin()` - Open ball feeder
- `closeBin()` - Close ball feeder
- `feedBall(delaySec)` - Automated feed sequence

---

### 📷 AprilTagSubsystem
Vision-based AprilTag detection for alliance color identification.

**Features:**
- Webcam integration for tag detection
- Real-time tag ID tracking
- Alliance color determination (Red/Blue)
- Global data storage for cross-OpMode access

**Key Methods:**
- `update()` - Process camera frames and detect tags
- `stop()` - Shutdown vision processor

**Detected Tags:**
- Tag 21 → Red Alliance
- Tag 22 → Blue Alliance  
- Tag 23 → Special pattern

**Usage:**
```java
tagSubsystem.update();
if (GlobalData.detectedTagId == 21) {
    // Red alliance autonomous
} else if (GlobalData.detectedTagId == 22) {
    // Blue alliance autonomous
}
```

---

## 🎮 OpModes

### Autonomous OpModes

#### Command-Based
- **AutoDawnCommand** - Advanced 11-path autonomous sequence
  - Preloaded specimen scoring
  - 3-cycle sample pickup and scoring
  - Percentage-based action triggers
  - AprilTag alliance detection
  - Turret alignment integration

#### Path-Based Autonomous (Pedro Pathing)
**Blue Bucket (BB):**
- AutoPath_BB1 through AutoPath_BB7

**Blue Specimen (BS):**
- AutoPath_BS1 through AutoPath_BS9

**Red Bucket (RB):**
- AutoPath_RB1 through AutoPath_RB7

**Red Specimen (RS):**
- AutoPath_RS1 through AutoPath_RS9

#### Specialized Autonomous
- **AprilTagPatternAuto** - AprilTag-based path selection
- **BallDetectionAuto** - Vision-based ball detection
- **ShooterAuto** - Shooter testing autonomous
- **MinervaAuto** - Main competition autonomous
- **GeneratedPathAuto** - Dynamically generated paths

### TeleOp OpModes
- **MinervaTeleOp** - Main driver control
- **TurretTeleOpTest** - Turret testing and tuning

## 🔧 Setup & Installation

### Prerequisites
- Android Studio (latest version recommended)
- FTC Robot Controller app on Control Hub/RC phone
- Driver Station app

### First-Time Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd MinervaRobot
   ```

2. **Sync Gradle**
   - Open project in Android Studio
   - Click "Sync Now" when prompted

3. **Full Install (First Time)**
   - Click the green Run button (not SlothLoad)
   - Wait for full APK installation (~30-40 seconds)
   - This installs all dependencies and the Sloth runtime

### Development Workflow with SlothLoad

After the initial full install, use SlothLoad for all subsequent deployments:

1. **Make code changes** in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

2. **Deploy with SlothLoad**
   - Use the SlothLoad button in Android Studio
   - Code deploys in **1-2 seconds** 🚀
   - No app restart required

3. **Test immediately**
   - Changes are live instantly
   - Select your OpMode and run

> **Note:** If you change dependencies in `build.gradle`, you must do a full install (not SlothLoad) to apply those changes.

## 🎯 Command-Based Programming

### Creating a New Command

```java
public class MyCommand extends CommandBase {
    @Override
    public void initialize() {
        // Setup code
    }

    @Override
    public void execute() {
        // Runs repeatedly
    }

    @Override
    public boolean isFinished() {
        return false; // Return true when done
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup code
    }
}
```

### Using Commands in OpModes

```java
@Autonomous(name = "My Auto")
public class MyAuto extends OpModeCommand {
    @Override
    public void initialize() {
        schedule(
            new MyCommand(),
            new FollowPath(follower, path1),
            new WaitCommand(500)
        );
    }
}
```

## 🛣️ Path Following with Pedro Pathing

### Creating Paths

```java
PathChain path = new PathBuilder(follower, Constants.pathConstraints)
    .addPath(new BezierLine(
        new Pose(startX, startY),
        new Pose(endX, endY)
    ))
    .setLinearHeadingInterpolation(startHeading, endHeading)
    .build();
```

### Percentage-Based Actions

```java
new FollowPath(follower, path)
    .alongWith(
        new SequentialCommandGroup(
            new WaitUntilPathPercent(follower, 0.50)
                .andThen(new InstantCommand(() -> actuator.runIntake())),
            new WaitUntilPathPercent(follower, 0.90)
                .andThen(new InstantCommand(() -> actuator.stopIntake()))
        )
    )
```

## 🐛 Troubleshooting

### "OpMode stuck in stop()" Error
This has been fixed with proper gamepad initialization and exception handling in `OpModeCommand.java`.

### ClassNotFoundException for Pedro Pathing
Ensure you've done a full install (not SlothLoad) after changing dependencies.

### SlothLoad Not Working
1. Verify Sloth is properly configured in `TeamCode/build.gradle`
2. Ensure you did a full install at least once
3. Check that code changes are in `org.firstinspires.ftc.teamcode` package

## 📝 Development Notes

### Sloth Hot-Reload Limitations
- Only reloads classes in `org.firstinspires.ftc.teamcode` package
- Library changes require full install
- `@Pinned` classes won't be hot-reloaded

### Best Practices
- Use SlothLoad for rapid iteration on teamcode
- Do full installs when changing dependencies
- Test autonomous routines incrementally
- Use telemetry extensively for debugging

## 🏆 Competition Ready

This codebase is designed for competitive FTC robotics with:
- Reliable autonomous routines
- Fast iteration during practice
- Modular, maintainable code structure
- Advanced vision and path following

## 📄 License

[Add your license here]

## 👥 Team

MinervaRobot FTC Team

---

**Built with ❤️ for FIRST Tech Challenge**
