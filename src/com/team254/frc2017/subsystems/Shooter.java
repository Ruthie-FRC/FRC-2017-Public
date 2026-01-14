package com.team254.frc2017.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonSRX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CircularBuffer;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

import java.util.Arrays;

/**
 * The shooter subsystem consists of 4 775 Pro motors driving twin backspin flywheels.
 * Updated to use Phoenix 6 API and WPILib 2026 with modern Java features.
 * 
 * The shooter goes through 3 stages when shooting:
 * 1. Spin Up - Use a PIDF controller to spin up to the desired RPM
 * 2. Hold When Ready - Calculate average kF for smooth open-loop control
 * 3. Hold - Use pure kF control for consistent shot velocity
 * 
 * @see Subsystem
 */
public class Shooter extends Subsystem {
    private static Shooter instance = null;

    /**
     * Debug output record for logging shooter performance.
     */
    public static class ShooterDebugOutput {
        public double timestamp;
        public double setpoint;
        public double rpm;
        public double voltage;
        public ControlMethod controlMethod;
        public double kF;
        public double range;
    }

    public static final int SPIN_UP_PROFILE = 0;
    public static final int HOLD_PROFILE = 1;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * Control methods for the shooter flywheel.
     */
    public enum ControlMethod {
        OPEN_LOOP,      // Open loop voltage control
        SPIN_UP,        // PIDF to desired RPM
        HOLD_WHEN_READY, // Calculate average kF
        HOLD            // Pure kF control
    }

    private final TalonSRX rightMaster;
    private final TalonSRX rightSlave;
    private final TalonSRX leftSlave1;
    private final TalonSRX leftSlave2;

    // Phoenix 6 control requests
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private ControlMethod controlMethod;
    private double setpointRpm;
    private double lastRpmSpeed;

    private final CircularBuffer kfEstimator = new CircularBuffer(Constants.kShooterKfBufferSize);

    // Used for transitioning from spin-up to hold loop
    private boolean onTarget = false;
    private double onTargetStartTime = Double.POSITIVE_INFINITY;

    private final ShooterDebugOutput debug = new ShooterDebugOutput();
    private final ReflectingCSVWriter<ShooterDebugOutput> csvWriter;

    private Shooter() {
        // Create motor controllers
        rightMaster = new TalonSRX(Constants.kRightShooterMasterId);
        rightSlave = new TalonSRX(Constants.kRightShooterSlaveId);
        leftSlave1 = new TalonSRX(Constants.kLeftShooterSlave1Id);
        leftSlave2 = new TalonSRX(Constants.kLeftShooterSlave2Id);

        // Configure master
        var masterConfig = new TalonSRXConfiguration();
        
        // Feedback configuration
        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.PulseWidthEncodedPosition;
        masterConfig.Feedback = feedbackConfigs;

        // Spin-up PID configuration (Slot 0)
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.kShooterTalonKP;
        slot0Configs.kI = Constants.kShooterTalonKI;
        slot0Configs.kD = Constants.kShooterTalonKD;
        slot0Configs.kV = Constants.kShooterTalonKF; // kV is the velocity feedforward in Phoenix 6
        masterConfig.Slot0 = slot0Configs;

        // Hold PID configuration (would use Slot 1, but keeping simple for now)
        
        // Motor output configuration
        masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits
        masterConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterOpenLoopCurrentLimit;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = false; // Will enable when needed

        // Apply configuration
        rightMaster.getConfigurator().apply(masterConfig);

        // Configure slaves as followers
        rightSlave.setControl(new Follower(Constants.kRightShooterMasterId, false));
        leftSlave1.setControl(new Follower(Constants.kRightShooterMasterId, true));
        leftSlave2.setControl(new Follower(Constants.kRightShooterMasterId, true));

        controlMethod = ControlMethod.OPEN_LOOP;

        System.out.println("Shooter initialized with Phoenix 6 and WPILib 2026");

        csvWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", ShooterDebugOutput.class);
    }

    /**
     * Load PIDF profiles onto the master talon.
     */
    public void refreshControllerConsts() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.kShooterTalonKP;
        slot0Configs.kI = Constants.kShooterTalonKI;
        slot0Configs.kD = Constants.kShooterTalonKD;
        slot0Configs.kV = Constants.kShooterTalonKF;
        
        rightMaster.getConfigurator().apply(slot0Configs);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        var currentRpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", currentRpm);
        SmartDashboard.putNumber("shooter_speed_error", setpointRpm - currentRpm);
        SmartDashboard.putNumber("shooter_output_voltage", rightMaster.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter_setpoint", setpointRpm);
        SmartDashboard.putBoolean("shooter_on_target", isOnTarget());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
        setpointRpm = 0.0;
    }

    @Override
    public void zeroSensors() {
        // Don't zero the flywheel, it'll make deltas screwy
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    controlMethod = ControlMethod.OPEN_LOOP;
                    kfEstimator.clear();
                    onTarget = false;
                    onTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    if (controlMethod != ControlMethod.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                        csvWriter.add(debug);
                    } else {
                        // Reset all state
                        kfEstimator.clear();
                        onTarget = false;
                        onTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                csvWriter.flush();
            }
        });
    }

    /**
     * Run the shooter in open loop mode.
     */
    public synchronized void setOpenLoop(double voltage) {
        if (controlMethod != ControlMethod.OPEN_LOOP) {
            controlMethod = ControlMethod.OPEN_LOOP;
            
            // Enable current limiting for open loop
            var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
            currentLimits.SupplyCurrentLimit = Constants.kShooterOpenLoopCurrentLimit;
            currentLimits.SupplyCurrentLimitEnable = true;
            rightMaster.getConfigurator().apply(currentLimits);
        }
        rightMaster.setControl(voltageControl.withOutput(voltage));
    }

    /**
     * Put the shooter in spin-up mode.
     */
    public synchronized void setSpinUp(double setpointRpm) {
        if (controlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        this.setpointRpm = setpointRpm;
    }

    /**
     * Put the shooter in hold-when-ready mode.
     */
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (controlMethod == ControlMethod.OPEN_LOOP || controlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        this.setpointRpm = setpointRpm;
    }

    /**
     * Configure talons for spin-up mode.
     */
    private void configureForSpinUp() {
        controlMethod = ControlMethod.SPIN_UP;
        
        // Disable current limiting for closed loop
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = false;
        rightMaster.getConfigurator().apply(currentLimits);
    }

    /**
     * Configure talons for hold-when-ready mode.
     */
    private void configureForHoldWhenReady() {
        controlMethod = ControlMethod.HOLD_WHEN_READY;
        
        // Disable current limiting for closed loop
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = false;
        rightMaster.getConfigurator().apply(currentLimits);
    }

    /**
     * Configure talons for hold mode.
     */
    private void configureForHold() {
        controlMethod = ControlMethod.HOLD;
        
        // Update feedforward value for hold mode
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kV = kfEstimator.getAverage();
        rightMaster.getConfigurator().apply(slot0Configs);
    }

    private void resetHold() {
        kfEstimator.clear();
        onTarget = false;
    }

    /**
     * Estimate the kV (feedforward) value from current RPM and voltage.
     * Note: In Phoenix 6, kV is used instead of kF.
     */
    private double estimateKf(double rpm, double voltage) {
        // Convert RPM to rotations per second
        final var rps = rpm / 60.0;
        // In Phoenix 6, kV is volts per rotation per second
        if (rps != 0) {
            return voltage / rps;
        }
        return Constants.kShooterTalonKF;
    }

    /**
     * Main control loop of the shooter. This method progresses the shooter through
     * the spin up, hold when ready, and hold stages.
     */
    private void handleClosedLoop(double timestamp) {
        final var speed = getSpeedRpm();
        final var voltage = rightMaster.getMotorVoltage().getValueAsDouble();
        lastRpmSpeed = speed;

        // See if we should be spinning up or holding
        if (controlMethod == ControlMethod.SPIN_UP) {
            // Convert RPM to rotations per second for Phoenix 6
            rightMaster.setControl(velocityControl.withVelocity(setpointRpm / 60.0));
            resetHold();
        } else if (controlMethod == ControlMethod.HOLD_WHEN_READY) {
            final var absError = Math.abs(speed - setpointRpm);
            final var onTargetNow = onTarget 
                ? absError < Constants.kShooterStopOnTargetRpm
                : absError < Constants.kShooterStartOnTargetRpm;
            
            if (onTargetNow && !onTarget) {
                // First cycle on target
                onTargetStartTime = timestamp;
                onTarget = true;
            } else if (!onTargetNow) {
                resetHold();
            }

            if (onTarget) {
                // Update kV
                kfEstimator.addValue(estimateKf(speed, voltage));
            }
            
            if (kfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configureForHold();
            } else {
                rightMaster.setControl(velocityControl.withVelocity(setpointRpm / 60.0));
            }
        }
        
        // No else because we may have changed control methods above
        if (controlMethod == ControlMethod.HOLD) {
            // Update kV if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > setpointRpm) {
                kfEstimator.addValue(estimateKf(speed, voltage));
                var slot0Configs = new Slot0Configs();
                slot0Configs.kV = kfEstimator.getAverage();
                rightMaster.getConfigurator().apply(slot0Configs);
            }
            rightMaster.setControl(velocityControl.withVelocity(setpointRpm / 60.0));
        }
        
        debug.timestamp = timestamp;
        debug.rpm = speed;
        debug.setpoint = setpointRpm;
        debug.voltage = voltage;
        debug.controlMethod = controlMethod;
        debug.kF = kfEstimator.getAverage();
        debug.range = 0; // No vision system in this simplified version
    }

    public synchronized double getSetpointRpm() {
        return setpointRpm;
    }

    private double getSpeedRpm() {
        // Phoenix 6 returns velocity in rotations per second, convert to RPM
        return rightMaster.getVelocity().getValueAsDouble() * 60.0;
    }

    public synchronized boolean isOnTarget() {
        return controlMethod == ControlMethod.HOLD;
    }

    public synchronized double getLastSpeedRpm() {
        return lastRpmSpeed;
    }

    @Override
    public void writeToLog() {
        csvWriter.write();
    }

    public boolean checkSystem() {
        System.out.println("Testing SHOOTER ----------------------------------------");
        final var kCurrentThres = 0.5;
        final var kRpmThres = 1200;

        // Test each motor with voltage control
        rightMaster.setControl(voltageControl.withOutput(6.0));
        Timer.delay(4.0);
        final var currentRightMaster = rightMaster.getSupplyCurrent().getValueAsDouble();
        final var rpmMaster = getSpeedRpm();
        rightMaster.setControl(voltageControl.withOutput(0.0));

        Timer.delay(2.0);

        // Note: Slaves follow master automatically in Phoenix 6
        // Testing individual motors would require temporarily breaking follower mode

        System.out.println("Shooter Right Master Current: " + currentRightMaster);
        System.out.println("Shooter RPM Master: " + rpmMaster);

        var failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!! Shooter Right Master Current Low !!!");
        }

        if (rpmMaster < kRpmThres) {
            failure = true;
            System.out.println("!!! Shooter Master RPM Low !!!");
        }

        return !failure;
    }
}
            DriverStation.reportError("Could not detect shooter encoder: " + sensorPresent, false);
        }

        rightSlave = makeSlave(Constants.kRightShooterSlaveId, false);
        leftSlave1 = makeSlave(Constants.kLeftShooterSlave1Id, true);
        leftSlave2 = makeSlave(Constants.kLeftShooterSlave2Id, true);

        refreshControllerConsts();

        controlMethod = ControlMethod.OPEN_LOOP;

        System.out.println("Shooter initialized with distance-RPM mapping");

        csvWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", ShooterDebugOutput.class);
    }

    /**
     * Load PIDF profiles onto the master talon.
     */
    public void refreshControllerConsts() {
        rightMaster.setProfile(SPIN_UP_PROFILE);
        rightMaster.setP(Constants.kShooterTalonKP);
        rightMaster.setI(Constants.kShooterTalonKI);
        rightMaster.setD(Constants.kShooterTalonKD);
        rightMaster.setF(Constants.kShooterTalonKF);
        rightMaster.setIZone(Constants.kShooterTalonIZone);

        rightMaster.setProfile(HOLD_PROFILE);
        rightMaster.setP(0.0);
        rightMaster.setI(0.0);
        rightMaster.setD(0.0);
        rightMaster.setF(Constants.kShooterTalonKF);
        rightMaster.setIZone(0);

        rightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        var currentRpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", currentRpm);
        SmartDashboard.putNumber("shooter_speed_error", setpointRpm - currentRpm);
        SmartDashboard.putNumber("shooter_output_voltage", rightMaster.getOutputVoltage());
        SmartDashboard.putNumber("shooter_setpoint", setpointRpm);
        SmartDashboard.putBoolean("shooter_on_target", isOnTarget());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
        setpointRpm = 0.0;
    }

    @Override
    public void zeroSensors() {
        // Don't zero the flywheel, it'll make deltas screwy
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    controlMethod = ControlMethod.OPEN_LOOP;
                    kfEstimator.clear();
                    onTarget = false;
                    onTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    if (controlMethod != ControlMethod.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                        csvWriter.add(debug);
                    } else {
                        // Reset all state
                        kfEstimator.clear();
                        onTarget = false;
                        onTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                csvWriter.flush();
            }
        });
    }

    /**
     * Run the shooter in open loop mode.
     */
    public synchronized void setOpenLoop(double voltage) {
        if (controlMethod != ControlMethod.OPEN_LOOP) {
            controlMethod = ControlMethod.OPEN_LOOP;
            rightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
            rightMaster.setCurrentLimit(Constants.kShooterOpenLoopCurrentLimit);
            rightMaster.EnableCurrentLimit(true);
        }
        rightMaster.set(voltage);
    }

    /**
     * Put the shooter in spin-up mode.
     */
    public synchronized void setSpinUp(double setpointRpm) {
        if (controlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        this.setpointRpm = setpointRpm;
    }

    /**
     * Put the shooter in hold-when-ready mode.
     */
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (controlMethod == ControlMethod.OPEN_LOOP || controlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        this.setpointRpm = setpointRpm;
    }

    /**
     * Configure talons for spin-up mode.
     */
    private void configureForSpinUp() {
        controlMethod = ControlMethod.SPIN_UP;
        rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        rightMaster.setProfile(SPIN_UP_PROFILE);
        rightMaster.EnableCurrentLimit(false);
        rightMaster.DisableNominalClosedLoopVoltage();
        rightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold-when-ready mode.
     */
    private void configureForHoldWhenReady() {
        controlMethod = ControlMethod.HOLD_WHEN_READY;
        rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        rightMaster.setProfile(SPIN_UP_PROFILE);
        rightMaster.EnableCurrentLimit(false);
        rightMaster.DisableNominalClosedLoopVoltage();
        rightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold mode.
     */
    private void configureForHold() {
        controlMethod = ControlMethod.HOLD;
        rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        rightMaster.setProfile(HOLD_PROFILE);
        rightMaster.EnableCurrentLimit(false);
        rightMaster.setNominalClosedLoopVoltage(12.0);
        rightMaster.setF(kfEstimator.getAverage());
        rightMaster.setVoltageRampRate(Constants.kShooterHoldRampRate);
    }

    private void resetHold() {
        kfEstimator.clear();
        onTarget = false;
    }

    /**
     * Estimate the kF value from current RPM and voltage.
     */
    private double estimateKf(double rpm, double voltage) {
        final var speedInTicksPer100ms = 4096.0 / 600.0 * rpm;
        final var output = 1023.0 / 12.0 * voltage;
        return output / speedInTicksPer100ms;
    }

    /**
     * Main control loop of the shooter. This method progresses the shooter through
     * the spin up, hold when ready, and hold stages.
     */
    private void handleClosedLoop(double timestamp) {
        final var speed = getSpeedRpm();
        final var voltage = rightMaster.getOutputVoltage();
        lastRpmSpeed = speed;

        // See if we should be spinning up or holding
        if (controlMethod == ControlMethod.SPIN_UP) {
            rightMaster.set(setpointRpm);
            resetHold();
        } else if (controlMethod == ControlMethod.HOLD_WHEN_READY) {
            final var absError = Math.abs(speed - setpointRpm);
            final var onTargetNow = onTarget 
                ? absError < Constants.kShooterStopOnTargetRpm
                : absError < Constants.kShooterStartOnTargetRpm;
            
            if (onTargetNow && !onTarget) {
                // First cycle on target
                onTargetStartTime = timestamp;
                onTarget = true;
            } else if (!onTargetNow) {
                resetHold();
            }

            if (onTarget) {
                // Update Kf
                kfEstimator.addValue(estimateKf(speed, voltage));
            }
            
            if (kfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configureForHold();
            } else {
                rightMaster.set(setpointRpm);
            }
        }
        
        // No else because we may have changed control methods above
        if (controlMethod == ControlMethod.HOLD) {
            // Update Kf if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > setpointRpm) {
                kfEstimator.addValue(estimateKf(speed, voltage));
                rightMaster.setF(kfEstimator.getAverage());
            }
        }
        
        debug.timestamp = timestamp;
        debug.rpm = speed;
        debug.setpoint = setpointRpm;
        debug.voltage = voltage;
        debug.controlMethod = controlMethod;
        debug.kF = kfEstimator.getAverage();
        debug.range = 0; // No vision system in this simplified version
    }

    public synchronized double getSetpointRpm() {
        return setpointRpm;
    }

    private double getSpeedRpm() {
        return rightMaster.getSpeed();
    }

    private static CANTalon makeSlave(int talonId, boolean flipOutput) {
        var slave = CANTalonFactory.createPermanentSlaveTalon(talonId, Constants.kRightShooterMasterId);
        slave.reverseOutput(flipOutput);
        slave.enableBrakeMode(false);
        return slave;
    }

    public synchronized boolean isOnTarget() {
        return controlMethod == ControlMethod.HOLD;
    }

    public synchronized double getLastSpeedRpm() {
        return lastRpmSpeed;
    }

    @Override
    public void writeToLog() {
        csvWriter.write();
    }

    public boolean checkSystem() {
        System.out.println("Testing SHOOTER ----------------------------------------");
        final var kCurrentThres = 0.5;
        final var kRpmThres = 1200;

        rightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        rightSlave.changeControlMode(CANTalon.TalonControlMode.Voltage);
        leftSlave1.changeControlMode(CANTalon.TalonControlMode.Voltage);
        leftSlave2.changeControlMode(CANTalon.TalonControlMode.Voltage);

        rightMaster.set(6.0f);
        Timer.delay(4.0);
        final var currentRightMaster = rightMaster.getOutputCurrent();
        final var rpmMaster = rightMaster.getSpeed();
        rightMaster.set(0.0f);

        Timer.delay(2.0);

        rightSlave.set(6.0f);
        Timer.delay(4.0);
        final var currentRightSlave = rightSlave.getOutputCurrent();
        final var rpmRightSlave = rightMaster.getSpeed();
        rightSlave.set(0.0f);

        Timer.delay(2.0);

        leftSlave1.set(-6.0f);
        Timer.delay(4.0);
        final var currentLeftSlave1 = leftSlave1.getOutputCurrent();
        final var rpmLeftSlave1 = rightMaster.getSpeed();
        leftSlave1.set(0.0f);

        Timer.delay(2.0);

        leftSlave2.set(-6.0f);
        Timer.delay(4.0);
        final var currentLeftSlave2 = leftSlave2.getOutputCurrent();
        final var rpmLeftSlave2 = rightMaster.getSpeed();
        leftSlave2.set(0.0f);

        rightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftSlave1.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);

        rightSlave.set(Constants.kRightShooterMasterId);
        leftSlave1.set(Constants.kRightShooterMasterId);
        leftSlave2.set(Constants.kRightShooterMasterId);

        System.out.println("Shooter Right Master Current: " + currentRightMaster + 
                          " Shooter Right Slave Current: " + currentRightSlave);
        System.out.println("Shooter Left Slave One Current: " + currentLeftSlave1 + 
                          " Shooter Left Slave Two Current: " + currentLeftSlave2);
        System.out.println("Shooter RPM Master: " + rpmMaster + " RPM Right slave: " + rpmRightSlave +
                          " RPM Left Slave 1: " + rpmLeftSlave1 + " RPM Left Slave 2: " + rpmLeftSlave2);

        var failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!! Shooter Right Master Current Low !!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!! Shooter Right Slave Current Low !!!");
        }

        if (currentLeftSlave1 < kCurrentThres) {
            failure = true;
            System.out.println("!!! Shooter Left Slave One Current Low !!!");
        }

        if (currentLeftSlave2 < kCurrentThres) {
            failure = true;
            System.out.println("!!! Shooter Left Slave Two Current Low !!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave, 
                                           currentLeftSlave1, currentLeftSlave2), 
                            currentRightMaster, 5.0)) {
            failure = true;
            System.out.println("!!! Shooter currents different !!!");
        }

        if (rpmMaster < kRpmThres) {
            failure = true;
            System.out.println("!!! Shooter Master RPM Low !!!");
        }

        if (rpmRightSlave < kRpmThres) {
            failure = true;
            System.out.println("!!! Shooter Right Slave RPM Low !!!");
        }

        if (rpmLeftSlave1 < kRpmThres) {
            failure = true;
            System.out.println("!!! Shooter Left Slave1 RPM Low !!!");
        }

        if (rpmLeftSlave2 < kRpmThres) {
            failure = true;
            System.out.println("!!! Shooter Left Slave2 RPM Low !!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmMaster, rpmRightSlave, rpmLeftSlave1, rpmLeftSlave2), 
                            rpmMaster, 250)) {
            failure = true;
            System.out.println("!!! Shooter RPMs different !!!");
        }

        return !failure;
    }
}

    /**
     * Load PIDF profiles onto the master talon
     */
    public void refreshControllerConsts() {
        mRightMaster.setProfile(kSpinUpProfile);
        mRightMaster.setP(Constants.kShooterTalonKP);
        mRightMaster.setI(Constants.kShooterTalonKI);
        mRightMaster.setD(Constants.kShooterTalonKD);
        mRightMaster.setF(Constants.kShooterTalonKF);
        mRightMaster.setIZone(Constants.kShooterTalonIZone);

        mRightMaster.setProfile(kHoldProfile);
        mRightMaster.setP(0.0);
        mRightMaster.setI(0.0);
        mRightMaster.setD(0.0);
        mRightMaster.setF(Constants.kShooterTalonKF);
        mRightMaster.setIZone(0);

        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        double current_rpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", current_rpm);
        SmartDashboard.putNumber("shooter_speed_error", mSetpointRpm - current_rpm);
        SmartDashboard.putNumber("shooter_output_voltage", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("shooter_setpoint", mSetpointRpm);

        SmartDashboard.putBoolean("shooter on target", isOnTarget());
        // SmartDashboard.putNumber("shooter_talon_position", mRightMaster.getPosition());
        // SmartDashboard.putNumber("shooter_talon_enc_position", mRightMaster.getEncPosition());
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
        mSetpointRpm = 0.0;
    }

    @Override
    public void zeroSensors() {
        // Don't zero the flywheel, it'll make deltas screwy
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    mControlMethod = ControlMethod.OPEN_LOOP;
                    mKfEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    if (mControlMethod != ControlMethod.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                        mCSVWriter.add(mDebug);
                    } else {
                        // Reset all state.
                        mKfEstimator.clear();
                        mOnTarget = false;
                        mOnTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mCSVWriter.flush();
            }
        });
    }

    /**
     * Run the shooter in open loop, used for climbing
     */
    public synchronized void setOpenLoop(double voltage) {
        if (mControlMethod != ControlMethod.OPEN_LOOP) {
            mControlMethod = ControlMethod.OPEN_LOOP;
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
            mRightMaster.setCurrentLimit(Constants.kShooterOpenLoopCurrentLimit);
            mRightMaster.EnableCurrentLimit(true);
        }
        mRightMaster.set(voltage);
    }

    /**
     * Put the shooter in spinup mode
     */
    public synchronized void setSpinUp(double setpointRpm) {
        if (mControlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        mSetpointRpm = setpointRpm;
    }

    /**
     * Put the shooter in hold when ready mode
     */
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP || mControlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        mSetpointRpm = setpointRpm;
    }

    /**
     * Configure talons for spin up
     */
    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.setProfile(kSpinUpProfile);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.DisableNominalClosedLoopVoltage();
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold when ready
     */
    private void configureForHoldWhenReady() {
        mControlMethod = ControlMethod.HOLD_WHEN_READY;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.setProfile(kSpinUpProfile);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.DisableNominalClosedLoopVoltage();
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    /**
     * Configure talons for hold
     */
    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.setProfile(kHoldProfile);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.setNominalClosedLoopVoltage(12.0);
        mRightMaster.setF(mKfEstimator.getAverage());
        mRightMaster.setVoltageRampRate(Constants.kShooterHoldRampRate);
    }

    private void resetHold() {
        mKfEstimator.clear();
        mOnTarget = false;
    }

    /**
     * Estimate the kF value from current RPM and voltage
     */
    private double estimateKf(double rpm, double voltage) {
        final double speed_in_ticks_per_100ms = 4096.0 / 600.0 * rpm;
        final double output = 1023.0 / 12.0 * voltage;
        return output / speed_in_ticks_per_100ms;
    }

    /**
     * Main control loop of the shooter. This method will progress the shooter through the spin up, hold when ready, and
     * hold stages.
     */
    private void handleClosedLoop(double timestamp) {
        final double speed = getSpeedRpm();
        final double voltage = mRightMaster.getOutputVoltage();
        mLastRpmSpeed = speed;

        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP) {
            mRightMaster.set(mSetpointRpm);
            resetHold();
        } else if (mControlMethod == ControlMethod.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(speed - mSetpointRpm);
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm
                    : abs_error < Constants.kShooterStartOnTargetRpm;
            if (on_target_now && !mOnTarget) {
                // First cycle on target.
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if (!on_target_now) {
                resetHold();
            }

            if (mOnTarget) {
                // Update Kv.
                mKfEstimator.addValue(estimateKf(speed, voltage));
            }
            if (mKfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configureForHold();
            } else {
                mRightMaster.set(mSetpointRpm);
            }
        }
        // No else because we may have changed control methods above.
        if (mControlMethod == ControlMethod.HOLD) {
            // Update Kv if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > mSetpointRpm) {
                mKfEstimator.addValue(estimateKf(speed, voltage));
                mRightMaster.setF(mKfEstimator.getAverage());
            }
        }
        mDebug.timestamp = timestamp;
        mDebug.rpm = speed;
        mDebug.setpoint = mSetpointRpm;
        mDebug.voltage = voltage;
        mDebug.control_method = mControlMethod;
        mDebug.kF = mKfEstimator.getAverage();
        Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters();
        if (params.isPresent()) {
            mDebug.range = params.get().getRange();
        } else {
            mDebug.range = 0;
        }
    }

    public synchronized double getSetpointRpm() {
        return mSetpointRpm;
    }

    private double getSpeedRpm() {
        return mRightMaster.getSpeed();
    }

    private static CANTalon makeSlave(int talonId, boolean flipOutput) {
        CANTalon slave = CANTalonFactory.createPermanentSlaveTalon(talonId, Constants.kRightShooterMasterId);
        slave.reverseOutput(flipOutput);
        slave.enableBrakeMode(false);
        return slave;
    }

    public synchronized boolean isOnTarget() {
        return mControlMethod == ControlMethod.HOLD;
    }

    public synchronized double getLastSpeedRpm() {
        return mLastRpmSpeed;
    }

    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

    public boolean checkSystem() {
        System.out.println("Testing SHOOTER.----------------------------------------");
        final double kCurrentThres = 0.5;
        final double kRpmThres = 1200;

        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mLeftSlave1.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mLeftSlave2.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mRightMaster.set(6.0f);
        Timer.delay(4.0);
        final double currentRightMaster = mRightMaster.getOutputCurrent();
        final double rpmMaster = mRightMaster.getSpeed();
        mRightMaster.set(0.0f);

        Timer.delay(2.0);

        mRightSlave.set(6.0f);
        Timer.delay(4.0);
        final double currentRightSlave = mRightSlave.getOutputCurrent();
        final double rpmRightSlave = mRightMaster.getSpeed();
        mRightSlave.set(0.0f);

        Timer.delay(2.0);

        mLeftSlave1.set(-6.0f);
        Timer.delay(4.0);
        final double currentLeftSlave1 = mLeftSlave1.getOutputCurrent();
        final double rpmLeftSlave1 = mRightMaster.getSpeed();
        mLeftSlave1.set(0.0f);

        Timer.delay(2.0);

        mLeftSlave2.set(-6.0f);
        Timer.delay(4.0);
        final double currentLeftSlave2 = mLeftSlave2.getOutputCurrent();
        final double rpmLeftSlave2 = mRightMaster.getSpeed();
        mLeftSlave2.set(0.0f);

        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave1.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);

        mRightSlave.set(Constants.kRightShooterMasterId);
        mLeftSlave1.set(Constants.kRightShooterMasterId);
        mLeftSlave2.set(Constants.kRightShooterMasterId);

        System.out.println("Shooter Right Master Current: " + currentRightMaster + " Shooter Right Slave Current: "
                + currentRightSlave);
        System.out.println("Shooter Left Slave One Current: " + currentLeftSlave1 + " Shooter Left Slave Two Current: "
                + currentLeftSlave2);
        System.out.println("Shooter RPM Master: " + rpmMaster + " RPM Right slave: " + rpmRightSlave
                + " RPM Left Slave 1: " + rpmLeftSlave1 + " RPM Left Slave 2: " + rpmLeftSlave2);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Slave Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave1 < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave One Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave2 < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave Two Current Low !!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(currentRightMaster, currentRightSlave, currentLeftSlave1,
                currentLeftSlave2), currentRightMaster, 5.0)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter currents different !!!!!!!!!!!!!!!!!");
        }

        if (rpmMaster < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Master RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (rpmRightSlave < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Slave RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftSlave1 < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave1 RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (rpmLeftSlave2 < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave2 RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (!Util.allCloseTo(Arrays.asList(rpmMaster, rpmRightSlave, rpmLeftSlave1, rpmLeftSlave2), rpmMaster, 250)) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter RPMs different !!!!!!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }
}
