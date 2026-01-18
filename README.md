# FRC 2017 - Shooter Subsystem (Modernized for WPILib 2026)

Team 254's 2017 FRC robot shooter subsystem code, fully modernized to use WPILib 2026 and Phoenix 6 with modern Java features (Java 17+). 

This repository has been streamlined to focus exclusively on the shooter mechanism, removing all other subsystems (drive, intake, hopper, feeder, gear grabber, LED, etc.) and supporting code (vision, autonomous paths, teleoperation controls).

## Modernization Changes

The code has been updated from legacy Java (Java 8) with deprecated WPILib/CTRE APIs to modern WPILib 2026:

- **WPILib 2026**: Updated from `IterativeRobot` to `TimedRobot`
- **Phoenix 6**: Migrated from deprecated `CANTalon` (Phoenix 5) to modern `TalonSRX` API
- **Records**: `ShooterAimingParameters` converted to a Java record for immutability
- **Var keyword**: Local variable type inference used throughout
- **Improved naming**: Modern naming conventions (removed Hungarian notation prefixes)
- **Better error handling**: Multi-catch blocks and improved exception messages
- **Cleaner code**: Removed deprecated patterns and simplified control flow

## Shooter Subsystem

The shooter consists of 4 775 Pro motors driving twin backspin flywheels. The shooter goes through 3 stages when shooting:

1. **Spin Up** - Use a PIDF controller to spin up to the desired RPM
2. **Hold When Ready** - Calculate average kV for smooth open-loop control  
3. **Hold** - Use pure kV control for consistent shot velocity

### Key Features

- Advanced closed-loop velocity control with adaptive feed-forward
- Three-stage shooting algorithm for optimal consistency
- CSV logging for performance analysis
- Self-test mode for system verification
- Phoenix 6 API with modern control requests

## Code Structure

```
src/
├── com/team254/frc2017/
│   ├── Robot.java                    # Main robot class (TimedRobot, shooter-only)
│   ├── Constants.java                # Shooter constants and tuning parameters
│   ├── ShooterAimingParameters.java  # Aiming parameter record
│   ├── loops/
│   │   ├── Loop.java                 # Loop interface
│   │   └── Looper.java               # Periodic task scheduler
│   └── subsystems/
│       ├── Shooter.java              # Main shooter subsystem (Phoenix 6)
│       └── Subsystem.java            # Base subsystem class
└── com/team254/lib/util/
    ├── CircularBuffer.java           # Circular buffer for kV estimation
    ├── ReflectingCSVWriter.java      # CSV logging utility
    ├── Util.java                     # General utilities
    └── ConstantsBase.java            # Base class for constants
```

## Building and Deploying

This code is designed for the FRC roboRIO with WPILib 2026 and CTRE Phoenix 6.

### Requirements
- Java 17 or later
- WPILib 2026+
- CTRE Phoenix 6 (for TalonSRX motor controllers)
- Gradle (included via WPILib)

### Build
```bash
./gradlew build
```

### Deploy to Robot
```bash
./gradlew deploy
```

## Hardware Configuration

### Motor Controllers
- Right Shooter Master: TalonSRX ID 2
- Right Shooter Slave: TalonSRX ID 1
- Left Shooter Slave 1: TalonSRX ID 13
- Left Shooter Slave 2: TalonSRX ID 14

### Sensors
- Encoder on master TalonSRX (pulse-width position)

## Constants and Tuning

Shooter parameters can be adjusted in `Constants.java`:

- **PIDF Gains**: `kShooterTalonKP`, `kShooterTalonKI`, `kShooterTalonKD`, `kShooterTalonKF` (now kV in Phoenix 6)
- **RPM Mapping**: `kFlywheelDistanceRpmValues` - distance to RPM lookup table
- **State Machine**: `kShooterStartOnTargetRpm`, `kShooterMinOnTargetSamples`

## API Changes (Phoenix 5 → Phoenix 6)

### Motor Control
- `CANTalon` → `TalonSRX` 
- `changeControlMode()` → Control request objects (`VoltageOut`, `VelocityVoltage`)
- `set()` → `setControl()`

### Configuration
- Monolithic setter methods → Configuration objects (`TalonSRXConfiguration`)
- `setP()`, `setI()`, etc. → `Slot0Configs` applied via `getConfigurator().apply()`

### Feedback
- `kF` (feed-forward) → `kV` (velocity feed-forward in volts per rotation per second)
- Units: RPM → Rotations per second

## License

See LICENSE file for details.

## Original Repository

This is a modernized, shooter-only version of Team 254's FRC 2017 robot code.  
Original repository: https://github.com/Team254/FRC-2017
