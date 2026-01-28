/**
 * @team: The Wire Clippers 5902
 * @name:    Constants.java
 * @purpose: A single place to store program-wide constants. Updated for 2026.
 * @name:    Daniel Sabalakov
 */
package org.frc5902.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.frc5902.robot.subsystems.drive.ModuleConfiguration;
import org.frc5902.robot.util.PID;

public final class Constants {
    public static final class RobotConstants {
        public static final String KITBOT_SERIAL_NUMBER = "";
        public static final String COMPBOT_SERIAL_NUMBER = "";
        public static final String TESTBENCH_SERIAL_NUMBER = "";

        // swap between either SIMULATOR or REPLAY
        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            REAL,
            SIM,
            REPLAY
        }

        public enum RobotType {
            KITBOT,
            COMP_V1,
            TESTBENCH;
        }

        public static int periodMs = 20;
        public static int DEFAULT_CURRENT_LIMIT = 60;

        public static final double robotMasskg = 100;
        public static final double robotMOI = 7;
        public static final double wheelCOF = 1.2;
        public static double MAX_SPEED_METERS_PER_SECOND = 4.0;
    }

    public static class PathPlannerConstants {

        // Pathplanner Constants
        public static DCMotor gearbox = DCMotor.getNEO(1);

        public static final RobotConfig ppConfig = new RobotConfig(
                RobotConstants.robotMasskg,
                RobotConstants.robotMOI,
                new ModuleConfig(
                        Units.inchesToMeters(DriveMotorConstants.driveWheelRadiusInches),
                        RobotConstants.MAX_SPEED_METERS_PER_SECOND,
                        RobotConstants.wheelCOF,
                        gearbox.withReduction(DriveMotorConstants.driveGearReduction),
                        RobotConstants.DEFAULT_CURRENT_LIMIT,
                        1),
                ModuleConfigurations.moduleTranslations);
    }

    public static class OdometryConstants {
        public static double odometryFrequency = 100.0;
    }

    public static class DriveMotorConstants {
        public static double driveWheelRadiusInches = 1.5; // convert inches
        // TODO CONVERT
        public static double driveGearReduction = 0.5;
        public static double drivePositionConversionFactor = 2 * Math.PI / Units.inchesToMeters(driveWheelRadiusInches);
        public static double driveVelocityConversionFactor = drivePositionConversionFactor / 60;
        public static int driveCurrentLimit = 40;
        public static double driveVoltageCompensation = 12;
        public static FeedbackSensor driveFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
        public static PID driveClosedLoop =
                PID.builder().proportional(0.2).deriviative(0.002).build();
        public static ResetMode driveResetMode = ResetMode.kNoResetSafeParameters;
        public static PersistMode drivePersistMode = PersistMode.kPersistParameters;
        public static IdleMode driveIdleMode = IdleMode.kBrake;

        // kS: volt to overcome static friction; kV: voltage to hold cruise velocity
        // TODO IMPLEMENT
        public static double kV = 0.0; // aka static
        public static double kS = 0.0; // aka velocity
    }

    public static class TurnMotorConstants {
        public static double turningRatio = 1;
        public static int turningCurrentLimit = 20;
        public static double turnVoltageCompensation = 12;
        public static FeedbackSensor turningFeedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static PID turnClosedLoop = PID.builder().proportional(0.0005).build();
        public static int turnEncoderResolution = 4096;
        public static double encoderToRadians = 2 * Math.PI / turnEncoderResolution;
        public static double turnPositionConversionFactor = 2 * Math.PI; // rotations => radians
        public static double turnVelocityConversionFactor = 2 * Math.PI / 60; // RPM => Radians / Sec
        public static ResetMode turnResetMode = ResetMode.kNoResetSafeParameters;
        public static PersistMode turnPersistMode = PersistMode.kPersistParameters;
        public static IdleMode turnIdleMode = IdleMode.kBrake;

        // IMPORTANT: Calculates shortest path...
        public static boolean turnPositionWrappingEnabled = true;
        public static double turnPIDMinInput = 0;
        public static double turnPIDMaxInput = 2 * Math.PI;
    }

    public class ModuleConfigurations {
        public double driveBaseWidthSQUARE = 24; // in
        public double driveBaseRadius = (Units.inchesToMeters(driveBaseWidthSQUARE) / 2) * Math.sqrt(2);
        // Front Left Module
        public ModuleConfiguration FrontLeftModule = ModuleConfiguration.builder()
                .DrivingID(23)
                .TurningID(13)
                .TurningMotorAbsoluteOffset(742)
                .DrivingMotorInverted(false)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(driveBaseRadius, driveBaseRadius))
                .build();

        // Front Right Module
        public ModuleConfiguration FrontRightModule = ModuleConfiguration.builder()
                .DrivingID(21)
                .TurningID(11)
                .TurningMotorAbsoluteOffset(305)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(driveBaseRadius, -driveBaseRadius))
                .build();
        // Back Left Module
        public ModuleConfiguration BackLeftModule = ModuleConfiguration.builder()
                .DrivingID(22)
                .TurningID(12)
                .TurningMotorAbsoluteOffset(829)
                .DrivingMotorInverted(false)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(-driveBaseRadius, driveBaseRadius))
                .build();
        // Back Right Module
        public ModuleConfiguration BackRightModule = ModuleConfiguration.builder()
                .DrivingID(20)
                .TurningID(10)
                .TurningMotorAbsoluteOffset(3673)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(true)
                .ModuleOffset(new Translation2d(-driveBaseRadius, -driveBaseRadius))
                .build();

        public Translation2d[] moduleTranslations = new Translation2d[] {
            FrontLeftModule.getModuleOffset(),
            FrontRightModule.getModuleOffset(),
            BackLeftModule.getModuleOffset(),
            BackRightModule.getModuleOffset()
        };
    }

    public static class DriveCommand_Constants {
        // TODO FIGURE OUT
        private static final double DEADBAND = 0.1;
        private static final double ANGLE_KP = 5.0;
        private static final double ANGLE_KD = 0.4;
        private static final double ANGLE_MAX_VELOCITY = 8.0;
        private static final double ANGLE_MAX_ACCELERATION = 20.0;
        private static final double FF_START_DELAY = 2.0; // Secs
        private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    }
}
