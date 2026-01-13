package com.team254.frc2017.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.frc2017.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. 
 * They are started when the robot powers up and stopped after the match.
 * Modernized with improved naming and structure.
 */
public class Looper {
    public final double period = Constants.kLooperDt;

    private boolean running;
    private final Notifier notifier;
    private final List<Loop> loops;
    private final Object taskRunningLock = new Object();
    private double timestamp = 0;
    private double dt = 0;

    private final Runnable runnable = () -> {
        synchronized (taskRunningLock) {
            if (running) {
                var now = Timer.getFPGATimestamp();

                for (var loop : loops) {
                    try {
                        loop.onLoop(now);
                    } catch (Exception e) {
                        System.err.println("Exception in loop: " + loop.getClass().getSimpleName());
                        e.printStackTrace();
                    }
                }

                dt = now - timestamp;
                timestamp = now;
            }
        }
    };

    public Looper() {
        notifier = new Notifier(runnable);
        running = false;
        loops = new ArrayList<>();
    }

    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock) {
            loops.add(loop);
        }
    }

    public synchronized void start() {
        if (!running) {
            System.out.println("Starting loops");
            synchronized (taskRunningLock) {
                timestamp = Timer.getFPGATimestamp();
                for (var loop : loops) {
                    loop.onStart(timestamp);
                }
                running = true;
            }
            notifier.startPeriodic(period);
        }
    }

    public synchronized void stop() {
        if (running) {
            System.out.println("Stopping loops");
            notifier.stop();
            synchronized (taskRunningLock) {
                running = false;
                timestamp = Timer.getFPGATimestamp();
                for (var loop : loops) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(timestamp);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt);
    }
}
