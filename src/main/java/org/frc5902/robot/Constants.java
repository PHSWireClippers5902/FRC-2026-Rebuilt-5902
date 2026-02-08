/**
 * @team: The Wire Clippers 5902
 * @name:    Constants.java
 * @purpose: A single place to store program-wide constants. Updated for 2026.
 * @name:    Daniel Sabalakov
 */
package org.frc5902.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
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

public class Constants {
    public static class RobotConstants {
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

        public static final double robotMasskg = 100; // change
        public static final double robotMOI = 7; // change  (sysid)
        public static final double wheelCOF = 1.2; // change (sysid)
        public static double MAX_SPEED_METERS_PER_SECOND = 4.0; // change

        // idk what a 'hal' is...
        public static boolean disableHAL = false;

        public static void disableHAL() {
            disableHAL = true;
        }
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

    public static class QuestConstants {
        public static double questFrequency = 100.0;
    }

    public static class DriveMotorConstants {
        public static double driveWheelRadiusInches = 1.5; // in inches // change
        // TODO CONVERT
        public static double driveGearReduction = 1.0 / 5.54; // change
        // First, convert to radians (2pi/5.54). Then, divide by drive train radius
        // Units.inchesToMeters(driveWheelRadiusInches)
        public static double drivePositionConversionFactor = 2 * Math.PI / 5.54; // // change
        public static double driveVelocityConversionFactor = drivePositionConversionFactor / 60; // change
        public static int driveCurrentLimit = 40;
        public static double driveVoltageCompensation = 12;
        public static FeedbackSensor driveFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
        public static PID driveClosedLoop =
                PID.builder().proportional(0.005).deriviative(0.002).build();
        public static ResetMode driveResetMode = ResetMode.kNoResetSafeParameters;
        public static PersistMode drivePersistMode = PersistMode.kPersistParameters;
        public static IdleMode driveIdleMode = IdleMode.kBrake;

        // kS: volt to overcome static friction; kV: voltage to hold cruise velocity
        // TODO IMPLEMENT
        public static double kV = 0.0; // aka velocity
        public static double kS = 0.0; // aka static
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

    public class ModuleConfigurations {
        public static double driveBaseWidthSQUARE = 24; // in // change
        public static double driveBaseRadius = (Units.inchesToMeters(driveBaseWidthSQUARE) / 2) * Math.sqrt(2);
        // Front Left Module
        public static ModuleConfiguration FrontLeftModule = ModuleConfiguration.builder()
                .DrivingID(10)
                .TurningID(20)
                .TurningEncoderID(30)
                .MagnetOffset(-0.081298828125)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(driveBaseRadius, driveBaseRadius))
                .build();

        // Front Right Module
        public static ModuleConfiguration FrontRightModule = ModuleConfiguration.builder()
                .DrivingID(11)
                .TurningID(21)
                .TurningEncoderID(31)
                .MagnetOffset(-0.099365234375)
                .DrivingMotorInverted(false)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(driveBaseRadius, -driveBaseRadius))
                .build();
        // Back Left Module
        public static ModuleConfiguration BackLeftModule = ModuleConfiguration.builder()
                .DrivingID(12)
                .TurningID(22)
                .TurningEncoderID(32)
                .MagnetOffset(0.18359375)
                .DrivingMotorInverted(true)
                .TurningMotorInverted(false)
                .ModuleOffset(new Translation2d(-driveBaseRadius, driveBaseRadius))
                .build();
        // Back Right Module
        public static ModuleConfiguration BackRightModule = ModuleConfiguration.builder()
                .DrivingID(13)
                .TurningID(23)
                .TurningEncoderID(33)
                .MagnetOffset(0.19384765625)
                .DrivingMotorInverted(false)
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

    public static class SimulatorConstants {
        public static DCMotor driveGearbox = DCMotor.getNEO(1);
        public static DCMotor turnGearbox = DCMotor.getNeo550(1);
        public static final double driveKp = 0.0;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.0;
        public static final double driveKv = 0.1;
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;
    }

    public static class IntakeConstants {
        public static final int FeederCANID = 40;
        public static final int IntakeCANID = 41;

        // is the feeder / intake inverted? We want anything intake to be positive
        public static final InvertType FeederInverted = InvertType.None;
        public static final InvertType IntakeInverted = InvertType.None;
    }
}
