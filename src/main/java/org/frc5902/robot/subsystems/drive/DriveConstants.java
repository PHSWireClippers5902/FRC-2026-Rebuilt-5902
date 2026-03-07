package org.frc5902.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.frc5902.robot.subsystems.drive.modules.ModuleConfiguration;
import org.frc5902.robot.util.motorutil.PID;
import org.frc5902.robot.util.swerve.ModuleLimits;

public class DriveConstants {
    public static class SimulatorConstants {
        public static DCMotor driveGearbox = DCMotor.getNEO(1);
        public static DCMotor turnGearbox = DCMotor.getNeo550(1);
        public static final double driveKp = 0.0;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.15;
        public static final double driveKv = 0.12;
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;
    }

    public class PhysicalConstraints {
        public static final double maxLinearSpeed = 6.0;
        public static final double maxAngularSpeed = 2.0 / ModuleConfigurations.driveBaseRadius;
        public static final double maxLinearAcceleration = 22.0;
        public static final double turnDeadbandDegrees = 0.3;
        public static final ModuleLimits moduleLimitsFree =
                new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));
    }

    public class ModuleConfigurations {
        public static double driveBaseWidthSQUARE = 27.125; // in // change
        public static double driveBaseRadius = (Units.inchesToMeters(driveBaseWidthSQUARE) / 2) * Math.sqrt(2);
        // WHEN I TUNE THIS, ALL WHEELS POINTING LEFT
        // THEN, USE VALUES FROM PHOENIX TUNER
        // Front Left Module
        public static ModuleConfiguration FrontLeftModule = ModuleConfiguration.builder()
                .DrivingID(10)
                .TurningID(20)
                .TurningEncoderID(30)
                .MagnetOffset(-0.07275390625)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(driveBaseRadius, driveBaseRadius))
                .build();

        // Front Right Module
        public static ModuleConfiguration FrontRightModule = ModuleConfiguration.builder()
                .DrivingID(11)
                .TurningID(21)
                .TurningEncoderID(31)
                .MagnetOffset(-0.07763671875)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(driveBaseRadius, -driveBaseRadius))
                .build();
        // Back Left Module
        public static ModuleConfiguration BackLeftModule = ModuleConfiguration.builder()
                .DrivingID(12)
                .TurningID(22)
                .TurningEncoderID(32)
                .MagnetOffset(-0.09130859375)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(-driveBaseRadius, driveBaseRadius))
                .build();
        // Back Right Module
        public static ModuleConfiguration BackRightModule = ModuleConfiguration.builder()
                .DrivingID(13)
                .TurningID(23)
                .TurningEncoderID(33)
                .MagnetOffset(0.41162109375)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(-driveBaseRadius, -driveBaseRadius))
                .build();

        public static Translation2d[] moduleTranslations = new Translation2d[] {
            FrontLeftModule.getModuleOffset(),
            FrontRightModule.getModuleOffset(),
            BackLeftModule.getModuleOffset(),
            BackRightModule.getModuleOffset()
        };
    }

    public static class DriveMotorConstants {
        public static double driveWheelRadiusInches = 1.422; // in inches // change
        // TODO CONVERT
        public static double driveGearReduction = 1.0 / 5.54; // change
        // First, convert to radians (2pi/5.54). Then, divide by drive train radius
        // Units.inchesToMeters(driveWheelRadiusInches)
        public static double drivePositionConversionFactor = 2 * Math.PI / 5.54; // // change
        public static double driveVelocityConversionFactor = drivePositionConversionFactor / 60; // change
        public static int driveFreeLimit = 50;
        public static int driveStallLimit = 70;
        public static double driveVoltageCompensation = 12;
        public static FeedbackSensor driveFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
        public static PID driveClosedLoop =
                PID.builder().proportional(0.0075).deriviative(0.002).build();
        public static ResetMode driveResetMode = ResetMode.kNoResetSafeParameters;
        public static PersistMode drivePersistMode = PersistMode.kPersistParameters;
        public static IdleMode driveIdleMode = IdleMode.kBrake;

        // kS: volt to overcome static friction; kV: voltage to hold cruise velocity
        public static double kV = 0.12; // aka velocity
        public static double kS = 0.15; // aka static
    }

    public static class TurnMotorConstants {
        public static double turningRatio = 1;
        public static int turningCurrentLimit = 20;
        public static double turnVoltageCompensation = 12;
        public static FeedbackSensor turningFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
        public static PID turnClosedLoop =
                PID.builder().proportional(0.4).deriviative(0.1).build();
        public static int turnEncoderResolution = 4096;
        public static boolean turnInverted = false;
        public static double turnPositionConversionFactor = 2 * Math.PI / (41.25); // to rads
        public static double turnPositionAbsoluteConversionFactor = 2 * Math.PI;
        public static double turnVelocityConversionFactor = turnPositionConversionFactor / 60; // RPM => Radians / Sec
        public static ResetMode turnResetMode = ResetMode.kNoResetSafeParameters;
        public static PersistMode turnPersistMode = PersistMode.kPersistParameters;
        public static IdleMode turnIdleMode = IdleMode.kCoast;

        // IMPORTANT: Calculates shortest path...
        public static boolean turnPositionWrappingEnabled = true;
        public static double turnPIDMinInput = 0;
        public static double turnPIDMaxInput = 2 * Math.PI;
    }

    public static class OdometryConstants {
        public static double odometryFrequency = 100.0;
    }
}
