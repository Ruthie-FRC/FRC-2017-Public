# Modernization Notes

This document describes the changes made to modernize the FRC 2017 codebase from legacy Java (Java 8) to modern Java (Java 17+).

## Overview

The original codebase contained a full robot implementation with multiple subsystems, vision processing, autonomous modes, and path following. This has been streamlined to focus exclusively on the **shooter subsystem** while modernizing the code to use contemporary Java features.

## Code Reduction

- **Original**: ~100+ Java files with complete robot functionality
- **Modernized**: 12 Java files (~2,500 lines) focused on shooter only
- **Removed**: Drive, Intake, Hopper, Feeder, LED, Gear Grabber, Vision, Auto modes, Path following

## Modernization Changes

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
CANTalon.FeedbackDeviceStatus sensorPresent = mRightMaster
    .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
```

**After**:
```java
var sensorPresent = rightMaster
    .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
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

### 4. Enhanced For-Each Loops

**Before**:
```java
for (Field field : mFields) {
    if (line.length() != 0) {
        line.append(", ");
    }
    line.append(field.getName());
}
```

**After**:
```java
for (var field : fields) {
    if (line.length() != 0) {
        line.append(", ");
    }
    line.append(field.getName());
}
```

### 5. Multi-Catch Exception Handling (Java 7+)

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

### 6. Final Class Modifiers for Utility Classes

**Before**:
```java
public class Util {
    private Util() {
    }
    // static methods
}
```

**After**:
```java
public final class Util {
    private Util() {
        throw new AssertionError("Utility class should not be instantiated");
    }
    // static methods
}
```

### 7. Constants as Static Final

**Before**:
```java
public static int kSpinUpProfile = 0;
public static int kHoldProfile = 1;
```

**After**:
```java
public static final int SPIN_UP_PROFILE = 0;
public static final int HOLD_PROFILE = 1;
```

### 8. Simplified Control Flow

**Before**:
```java
public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
        result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
}
```

**After**:
```java
public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
    for (var valueIn : list) {
        if (!epsilonEquals(valueIn, value, epsilon)) {
            return false;
        }
    }
    return true;
}
```

## Removed Dependencies

- **RobotState**: Vision-based state tracking removed
- **CrashTracker/CrashTrackingRunnable**: Replaced with standard exception handling
- **Math utilities**: Rotation2d, Translation2d, RigidTransform2d (not needed for shooter-only)
- **Control libraries**: Path following, pure pursuit, motion profiling
- **Hardware drivers**: NavX, LIDAR, ultrasonic sensors, pressure sensors

## Retained Core Functionality

The shooter subsystem retains all its original functionality:

- **Three-stage shooting algorithm**: Spin up → Hold when ready → Hold
- **Adaptive feed-forward control**: kF estimation for consistent velocity
- **CSV logging**: Performance data recording
- **Self-test mode**: Motor current and RPM verification
- **Distance-based RPM mapping**: Auto-aiming lookup table

## File Structure

```
src/
├── com/team254/frc2017/
│   ├── Robot.java                    # Simplified shooter-only robot
│   ├── Constants.java                # Shooter-specific constants only
│   ├── ShooterAimingParameters.java  # Record for aiming data
│   ├── loops/
│   │   ├── Loop.java                 # Periodic task interface
│   │   └── Looper.java               # Task scheduler (modernized)
│   └── subsystems/
│       ├── Shooter.java              # Main shooter logic (modernized)
│       └── Subsystem.java            # Base class
└── com/team254/lib/util/
    ├── CircularBuffer.java           # Modernized utility
    ├── ReflectingCSVWriter.java      # Modernized logging
    ├── Util.java                     # Modernized helpers
    ├── ConstantsBase.java            # Config file support
    └── drivers/
        └── CANTalonFactory.java      # Motor controller factory
```

## Testing and Deployment

The code maintains compatibility with:
- WPILib (FRC robotics library)
- CTRE Phoenix (CANTalon motor controllers)
- roboRIO hardware platform

**Note**: This code requires a complete FRC development environment (WPILib, Phoenix libraries) to compile and deploy. The modernized code should compile successfully with Java 17+ and the appropriate FRC libraries installed.

## Benefits of Modernization

1. **Improved Readability**: Modern syntax is more concise and easier to understand
2. **Type Safety**: Records provide immutable data with compile-time safety
3. **Less Boilerplate**: var keyword reduces verbosity while maintaining type safety
4. **Better Performance**: Modern JVM optimizations for newer language features
5. **Maintainability**: Cleaner code is easier to modify and extend
6. **Industry Standards**: Aligns with current Java best practices

## Future Improvements

Potential further modernizations:
- Pattern matching (Java 16+)
- Text blocks for multi-line strings (Java 15+)
- Sealed classes for control methods (Java 17+)
- Switch expressions (Java 14+)
- Stream API for collection processing

## Compatibility

- **Minimum Java Version**: Java 17
- **Target Platform**: roboRIO (with updated JRE)
- **Libraries**: WPILib 2024+, CTRE Phoenix 5.x or Phoenix 6

---

**Original Code**: Team 254 FRC 2017 (Misfire)  
**Modernization**: 2024  
**Focus**: Shooter subsystem only with modern Java features
