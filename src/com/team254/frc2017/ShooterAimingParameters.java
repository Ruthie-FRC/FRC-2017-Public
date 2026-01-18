package com.team254.frc2017;

/**
 * A container record to specify the shooter aiming parameters.
 * Modernized to use Java record syntax.
 * 
 * @param range The distance to target in inches
 * @param robotToGoalAngle The angle from robot to goal in degrees
 * @param lastSeenTimestamp The timestamp when the target was last seen
 * @param stability The stability metric for the target tracking
 */
public record ShooterAimingParameters(
    double range,
    double robotToGoalAngle,
    double lastSeenTimestamp,
    double stability
) {
    // Records automatically generate constructor, getters, equals, hashCode, and toString
}
