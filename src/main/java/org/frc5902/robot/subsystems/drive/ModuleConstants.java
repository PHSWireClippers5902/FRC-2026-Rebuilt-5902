package org.frc5902.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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
    public static class SwervePIDConstants {
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = true;
    }

    @AutoLog
    public static final class AbsoluteChange {
        // 25 742
        // 23 328
        // 24 924
        // 22 3677
        public static final int FrontLeftChange = 742;
        public static final int FrontRightChange = 305;
        public static final int BackLeftChange = 829;
        public static final int BackRightChange = 3673;
    }

    @AutoLog
    public static final class SwerveCANConstants {
        public static final int kFrontLeftDrivingCanId = 23;
        public static final int kRearLeftDrivingCanId = 22;
        public static final int kFrontRightDrivingCanId = 21;
        public static final int kRearRightDrivingCanId = 20;

        public static final int kFrontLeftTurningCanId = 13;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 11;
        public static final int kRearRightTurningCanId = 10;
    }
}
