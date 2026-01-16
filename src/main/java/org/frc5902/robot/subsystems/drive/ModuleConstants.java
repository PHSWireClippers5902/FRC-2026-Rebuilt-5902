package org.frc5902.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import edu.wpi.first.math.geometry.Translation2d;
import org.frc5902.robot.util.PID;
import org.littletonrobotics.junction.AutoLog;

public class ModuleConstants {
    @AutoLog
    public static class DriveConfigurations {
        public static double driveWheelRadius = 0.0379; // convert m
        public static double driveGearReduction = 0.5;
        public static double drivePositionConversionFactor = 2 * Math.PI / driveWheelRadius;
        public static double driveVelocityConversionFactor = drivePositionConversionFactor / 60;
        public static double driveCurrentLimit = 40;
        public static FeedbackSensor driveFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
        public static PID driveClosedLoop = PID.builder("Swerve Drive Closed Loop")
                .proportional(0.2)
                .integral(0.002)
                .build();
        public ResetMode driveResetMode = ResetMode.kNoResetSafeParameters;
        public PersistMode drivePersistMode = PersistMode.kPersistParameters;
    }

    @AutoLog
    public static class TurnConfigurations {
        public static double turningRatio = 1;
        public static double maxTurningSpeed = 2 * Math.PI; // 1 rotation per second
        public static double turningCurrentLimit = 20;
        public static FeedbackSensor turningFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static PID turnClosedLoop =
                PID.builder("Swerve Drive Closed Loop").proportional(0.0005).build();
        public static int turnEncoderResolution = 4096;
        public static double encoderToRadians = 2 * Math.PI / turnEncoderResolution;
        public static double turnPositionConversionFactor = 4096;
        public ResetMode turnResetMode = ResetMode.kNoResetSafeParameters;
        public PersistMode turnPersistMode = PersistMode.kPersistParameters;
    }

    @AutoLog
    public static class ModuleConfigurations {
        // Front Left Module
        public static ModuleConfiguration FrontLeftModule = ModuleConfiguration.builder("FrontLeft")
                .DrivingID(23)
                .TurningID(13)
                .TurningMotorAbsoluteOffset(742)
                .DrivingMotorInverted(false)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(0.523, 0.523))
                .build();
        // Front Right Module
        public static ModuleConfiguration FrontRightModule = ModuleConfiguration.builder("FrontLeft")
                .DrivingID(21)
                .TurningID(11)
                .TurningMotorAbsoluteOffset(305)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(0.523, -0.523))
                .build();
        // Back Left Module
        public static ModuleConfiguration BackLeftModule = ModuleConfiguration.builder("BackLeft")
                .DrivingID(22)
                .TurningID(12)
                .TurningMotorAbsoluteOffset(829)
                .DrivingMotorInverted(false)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(-0.523, 0.523))
                .build();
        // Back Right Module
        public static ModuleConfiguration BackRightModule = ModuleConfiguration.builder("BackRight")
                .DrivingID(20)
                .TurningID(10)
                .TurningMotorAbsoluteOffset(3673)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(-0.523, -0.523))
                .build();
    }
}
