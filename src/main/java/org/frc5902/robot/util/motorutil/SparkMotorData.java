/**
 * @team: The Wire Clippers 5902
 * @name:    SparkMotorData
 * @purpose: A potential abstract SparkMotorData object that might provide even better abstraction
 * @author:    Daniel Sabalakov
 */
package org.frc5902.robot.util.motorutil;

public record SparkMotorData(
        boolean connected,
        double positionRadians,
        double velocityRadiansPerSecond,
        double appliedVoltage,
        double supplyCurrentAmps,
        double temperatureCelsius) {}
