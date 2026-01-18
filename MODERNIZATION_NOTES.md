# Modernization Notes

This document describes the changes made to modernize the FRC 2017 codebase from legacy Java (Java 8) with deprecated APIs to modern Java (Java 17+) with WPILib 2026 and Phoenix 6.

## Overview

The original codebase contained a full robot implementation with multiple subsystems, vision processing, autonomous modes, and path following. This has been streamlined to focus exclusively on the **shooter subsystem** while modernizing the code to use contemporary Java features and the latest WPILib/CTRE APIs.

## Code Reduction

- **Original**: ~100+ Java files with complete robot functionality
- **Modernized**: 11 Java files (~2,500 lines) focused on shooter only
- **Removed**: Drive, Intake, Hopper, Feeder, LED, Gear Grabber, Vision, Auto modes, Path following

## WPILib 2026 API Updates

### 1. TimedRobot (Replacing IterativeRobot)

**Before** (Deprecated IterativeRobot):
```java
public class Robot extends IterativeRobot {
    public Robot() {
        System.out.println("Robot constructed");
    }
    
    @Override
    public void robotInit() { ... }
}
```

**After** (WPILib 2026 TimedRobot):
```java
public class Robot extends TimedRobot {
    public Robot() {
        super();  // Call TimedRobot constructor
        System.out.println("Robot constructed");
    }
    
    @Override
    public void robotInit() { ... }
}
```

### 2. Phoenix 6 API (Replacing Phoenix 5 CANTalon)

**Before** (Phoenix 5 - Deprecated):
```java
import com.ctre.CANTalon;

private final CANTalon rightMaster;

rightMaster = new CANTalon(Constants.kRightShooterMasterId);
rightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
rightMaster.set(voltage);
```

**After** (Phoenix 6 - Modern):
```java
import com.ctre.phoenix6.hardware.TalonSRX;
import com.ctre.phoenix6.controls.VoltageOut;

private final TalonSRX rightMaster;
private final VoltageOut voltageControl = new VoltageOut(0);

rightMaster = new TalonSRX(Constants.kRightShooterMasterId);
rightMaster.setControl(voltageControl.withOutput(voltage));
```

### 3. Configuration System (Phoenix 6)

**Before** (Phoenix 5 - Method calls):
```java
rightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
rightMaster.reverseSensor(true);
rightMaster.enableBrakeMode(false);
rightMaster.setP(Constants.kShooterTalonKP);
rightMaster.setI(Constants.kShooterTalonKI);
rightMaster.setD(Constants.kShooterTalonKD);
rightMaster.setF(Constants.kShooterTalonKF);
```

**After** (Phoenix 6 - Configuration objects):
```java
var masterConfig = new TalonSRXConfiguration();

// Feedback configuration
var feedbackConfigs = new FeedbackConfigs();
feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.PulseWidthEncodedPosition;
masterConfig.Feedback = feedbackConfigs;

// PID configuration
var slot0Configs = new Slot0Configs();
slot0Configs.kP = Constants.kShooterTalonKP;
slot0Configs.kI = Constants.kShooterTalonKI;
slot0Configs.kD = Constants.kShooterTalonKD;
slot0Configs.kV = Constants.kShooterTalonKF; // kF → kV
masterConfig.Slot0 = slot0Configs;

// Apply all configuration at once
rightMaster.getConfigurator().apply(masterConfig);
```

### 4. Velocity Control Units (Phoenix 6)

**Before** (Phoenix 5 - RPM directly):
```java
rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
rightMaster.set(setpointRpm);  // Set in RPM
```

**After** (Phoenix 6 - Rotations per second):
```java
private final VelocityVoltage velocityControl = new VelocityVoltage(0);

// Convert RPM to rotations per second
rightMaster.setControl(velocityControl.withVelocity(setpointRpm / 60.0));
```

### 5. Follower Mode (Phoenix 6)

**Before** (Phoenix 5):
```java
CANTalon slave = new CANTalon(slaveId);
slave.changeControlMode(CANTalon.TalonControlMode.Follower);
slave.set(masterId);
slave.reverseOutput(inverted);
```

**After** (Phoenix 6):
```java
TalonSRX slave = new TalonSRX(slaveId);
slave.setControl(new Follower(masterId, inverted));
```

### 6. Sensor Reading (Phoenix 6)

**Before** (Phoenix 5):
```java
double rpm = rightMaster.getSpeed();  // Returns RPM
double voltage = rightMaster.getOutputVoltage();
double current = rightMaster.getOutputCurrent();
```

**After** (Phoenix 6):
```java
// Returns rotations per second, convert to RPM
double rpm = rightMaster.getVelocity().getValueAsDouble() * 60.0;
double voltage = rightMaster.getMotorVoltage().getValueAsDouble();
double current = rightMaster.getSupplyCurrent().getValueAsDouble();
```

### 7. kF to kV Conversion

In Phoenix 6, the feedforward term changed from `kF` to `kV` with different units:

**Phoenix 5 kF**: Arbitrary units based on encoder ticks per 100ms  
**Phoenix 6 kV**: Volts per rotation per second

**Conversion**:
```java
// Old kF calculation (Phoenix 5)
private double estimateKf(double rpm, double voltage) {
    final double speedInTicksPer100ms = 4096.0 / 600.0 * rpm;
    final double output = 1023.0 / 12.0 * voltage;
    return output / speedInTicksPer100ms;
}

// New kV calculation (Phoenix 6)
private double estimateKf(double rpm, double voltage) {
    final var rps = rpm / 60.0;  // Convert to rotations per second
    if (rps != 0) {
        return voltage / rps;  // Volts per rotation per second
    }
    return Constants.kShooterTalonKF;
}
```

## Modern Java Features Applied

### 1. Records (Java 14+)

**Before** (ShooterAimingParameters.java):
```java
public class ShooterAimingParameters {
    double range;
    double last_seen_timestamp;
    double stability;
    Rotation2d robot_to_goal;

    public ShooterAimingParameters(double range, Rotation2d robot_to_goal, 
                                  double last_seen_timestamp, double stability) {
        this.range = range;
        this.robot_to_goal = robot_to_goal;
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
    }

    public double getRange() { return range; }
    // ... more getters
}
```

**After**:
```java
public record ShooterAimingParameters(
    double range,
    double robotToGoalAngle,
    double lastSeenTimestamp,
    double stability
) {
    // Automatic constructor, getters, equals, hashCode, toString
}
```

### 2. Local Variable Type Inference (Java 10+)

**Before**:
```java
TalonSRXConfiguration masterConfig = new TalonSRXConfiguration();
FeedbackDeviceStatus sensorPresent = rightMaster.isSensorPresent(...);
```

**After**:
```java
var masterConfig = new TalonSRXConfiguration();
var sensorPresent = rightMaster.isSensorPresent(...);
```

### 3. Improved Naming Conventions

**Before** (Hungarian notation):
```java
private boolean mOnTarget = false;
private double mSetpointRpm;
private CircularBuffer mKfEstimator;
```

**After** (Modern Java conventions):
```java
private boolean onTarget = false;
private double setpointRpm;
private final CircularBuffer kfEstimator;
```

### 4. Multi-Catch Exception Handling (Java 7+)

**Before**:
```java
try {
    line.append(field.get(value).toString());
} catch (IllegalArgumentException e) {
    e.printStackTrace();
} catch (IllegalAccessException e) {
    e.printStackTrace();
}
```

**After**:
```java
try {
    line.append(field.get(value).toString());
} catch (IllegalArgumentException | IllegalAccessException e) {
    System.err.println("Error accessing field: " + field.getName());
    e.printStackTrace();
}
```

## Removed Dependencies

- **RobotState**: Vision-based state tracking removed
- **CrashTracker/CrashTrackingRunnable**: Replaced with standard exception handling
- **Math utilities**: Rotation2d, Translation2d, RigidTransform2d (not needed for shooter-only)
- **Control libraries**: Path following, pure pursuit, motion profiling
- **Hardware drivers**: NavX, LIDAR, ultrasonic sensors, pressure sensors
- **CANTalonFactory**: No longer needed with Phoenix 6 direct instantiation

## Retained Core Functionality

The shooter subsystem retains all its original functionality:

- **Three-stage shooting algorithm**: Spin up → Hold when ready → Hold
- **Adaptive feed-forward control**: kV estimation for consistent velocity
- **CSV logging**: Performance data recording
- **Self-test mode**: Motor current and RPM verification
- **Distance-based RPM mapping**: Auto-aiming lookup table

## File Structure

```
src/
├── com/team254/frc2017/
│   ├── Robot.java                    # TimedRobot with shooter-only logic
│   ├── Constants.java                # Shooter-specific constants only
│   ├── ShooterAimingParameters.java  # Record for aiming data
│   ├── loops/
│   │   ├── Loop.java                 # Periodic task interface
│   │   └── Looper.java               # Task scheduler (modernized)
│   └── subsystems/
│       ├── Shooter.java              # Phoenix 6 shooter logic
│       └── Subsystem.java            # Base class
└── com/team254/lib/util/
    ├── CircularBuffer.java           # Modernized utility
    ├── ReflectingCSVWriter.java      # Modernized logging
    ├── Util.java                     # Modernized helpers
    └── ConstantsBase.java            # Config file support
```

## Benefits of Modernization

1. **Latest APIs**: WPILib 2026 and Phoenix 6 with better performance and features
2. **Type Safety**: Records provide immutable data with compile-time safety
3. **Less Boilerplate**: var keyword and records reduce verbosity
4. **Better Configuration**: Phoenix 6's configuration objects are more maintainable
5. **Improved Units**: Phoenix 6 uses SI units (rotations per second vs ticks)
6. **Future-Proof**: Uses current FRC standards and best practices

## Migration Summary

| Component | Before | After |
|-----------|--------|-------|
| Robot Base Class | IterativeRobot | TimedRobot |
| Motor Controller | CANTalon (Phoenix 5) | TalonSRX (Phoenix 6) |
| Control Method | `set()` | `setControl()` with request objects |
| Configuration | Individual setters | Configuration objects |
| Velocity Units | RPM | Rotations per second |
| Feedforward | kF (arbitrary units) | kV (volts/rps) |
| Follower Setup | Control mode + set | Follower control request |

## Compatibility

- **Minimum Java Version**: Java 17
- **Target Platform**: roboRIO with WPILib 2026
- **Libraries**: WPILib 2026, CTRE Phoenix 6

---

**Original Code**: Team 254 FRC 2017 (Misfire)  
**Modernization**: 2024-2026  
**Focus**: Shooter subsystem only with WPILib 2026 and Phoenix 6
