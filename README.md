# FRC 2017 - Shooter Subsystem (Modernized)

Team 254's 2017 FRC robot shooter subsystem code, modernized to use modern Java features (Java 17+). 

This repository has been streamlined to focus exclusively on the shooter mechanism, removing all other subsystems (drive, intake, hopper, feeder, gear grabber, LED, etc.) and supporting code (vision, autonomous paths, teleoperation controls).

## Modernization Changes

The code has been updated from legacy Java (Java 8) to modern Java with the following improvements:

- **Records**: `ShooterAimingParameters` converted to a Java record for immutability
- **Var keyword**: Local variable type inference used throughout
- **Improved naming**: Modern naming conventions (removed Hungarian notation prefixes)
- **Better error handling**: Multi-catch blocks and improved exception messages
- **Cleaner code**: Removed deprecated patterns and simplified control flow

## Shooter Subsystem

The shooter consists of 4 775 Pro motors driving twin backspin flywheels. The shooter goes through 3 stages when shooting:

1. **Spin Up** - Use a PIDF controller to spin up to the desired RPM
2. **Hold When Ready** - Calculate average kF for smooth open-loop control  
3. **Hold** - Use pure kF control for consistent shot velocity

### Key Features

- Advanced closed-loop velocity control with adaptive feed-forward
- Three-stage shooting algorithm for optimal consistency
- CSV logging for performance analysis
- Self-test mode for system verification

## Code Structure

```
src/
├── com/team254/frc2017/
│   ├── Robot.java                    # Main robot class (shooter-only)
│   ├── Constants.java                # Shooter constants and tuning parameters
│   ├── ShooterAimingParameters.java  # Aiming parameter record
│   ├── loops/
│   │   ├── Loop.java                 # Loop interface
│   │   └── Looper.java               # Periodic task scheduler
│   └── subsystems/
│       ├── Shooter.java              # Main shooter subsystem
│       └── Subsystem.java            # Base subsystem class
└── com/team254/lib/util/
    ├── CircularBuffer.java           # Circular buffer for kF estimation
    ├── ReflectingCSVWriter.java      # CSV logging utility
    ├── Util.java                     # General utilities
    ├── ConstantsBase.java            # Base class for constants
    └── drivers/
        └── CANTalonFactory.java      # CANTalon configuration factory
```

## Building and Deploying

This code is designed for the FRC roboRIO with the WPILib framework and CTRE Phoenix libraries.

### Requirements
- Java 17 or later
- WPILib (FRC libraries)
- CTRE Phoenix (for CANTalon motor controllers)

### Build
```bash
ant build
```

### Deploy to Robot
```bash
ant deploy
```

## Hardware Configuration

### Motor Controllers
- Right Shooter Master: Talon ID 2
- Right Shooter Slave: Talon ID 1
- Left Shooter Slave 1: Talon ID 13
- Left Shooter Slave 2: Talon ID 14

### Sensors
- CTRE Mag Encoder on master talon

## Constants and Tuning

Shooter parameters can be adjusted in `Constants.java`:

- **PIDF Gains**: `kShooterTalonKP`, `kShooterTalonKI`, `kShooterTalonKD`, `kShooterTalonKF`
- **RPM Mapping**: `kFlywheelDistanceRpmValues` - distance to RPM lookup table
- **State Machine**: `kShooterStartOnTargetRpm`, `kShooterMinOnTargetSamples`

## License

See LICENSE file for details.

## Original Repository

This is a modernized, shooter-only version of Team 254's FRC 2017 robot code.  
Original repository: https://github.com/Team254/FRC-2017
