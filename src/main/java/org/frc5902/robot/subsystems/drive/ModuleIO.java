package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRadians = 0.0;
        public double driveVelocityRadiansPerSecond = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadiansPerSecond = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRadians = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
        public Rotation2d odometryAbsoluteTurnPositions = new Rotation2d();
    }

    // loggable inputs
    public default void updateInputs(ModuleIOInputs inputs) {}
    // sets drive motor at the specific open loop value
    public default void setDriveOpenLoop(double output) {}
    // set turn open loop at specified open loop value
    public default void setTurnOpenLoop(double output) {}
    // set drive motor at velocity
    public default void setDriveVelocity(double velocityRadiansPerSecond) {}
    // sets motor specific position
    public default void setTurnPosition(Rotation2d rotation) {}
    // sends relative encoder update to absolute encoders
    public default void setRelativeToAbsoluteValues() {}
    ;
}
