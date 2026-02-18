/**
 * Describes Desired Motor Data from a base Spark Max...
 */
package org.frc5902.robot.util;

public record SparkMotorData(
        boolean connected,
        double positionRadians,
        double velocityRadiansPerSecond,
        double appliedVoltage,
        double supplyCurrentAmps,
        double temperatureCelsius) {}
