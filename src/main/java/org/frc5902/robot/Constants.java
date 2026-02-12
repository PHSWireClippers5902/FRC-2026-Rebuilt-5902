/**
 * @team: The Wire Clippers 5902
 * @name:    Constants.java
 * @purpose: A single place to store program-wide constants. Updated for 2026.
 * @name:    Daniel Sabalakov
 */
package org.frc5902.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.frc5902.robot.subsystems.drive.DriveConstants;
import org.frc5902.robot.subsystems.drive.DriveConstants.ModuleConfigurations;

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
        public static double loopPeriodSeconds = 1.0 / (double) periodMs;
        public static int DEFAULT_CURRENT_LIMIT = 60;

        public static final double robotMasskg = 100; // change
        public static final double robotMOI = 7; // change  (sysid)
        public static final double wheelCOF = 1.2; // change (sysid)
        public static double MAX_SPEED_METERS_PER_SECOND = 4.0; // change
        public static boolean tuningMode = true;
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
                        Units.inchesToMeters(DriveConstants.DriveMotorConstants.driveWheelRadiusInches),
                        RobotConstants.MAX_SPEED_METERS_PER_SECOND,
                        RobotConstants.wheelCOF,
                        gearbox.withReduction(DriveConstants.DriveMotorConstants.driveGearReduction),
                        RobotConstants.DEFAULT_CURRENT_LIMIT,
                        1),
                ModuleConfigurations.moduleTranslations);
    }

    public static class QuestConstants {
        public static double questFrequency = 100.0;
        public static Transform3d ROBOT_TO_QUEST = new Transform3d(
                new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(12), Units.inchesToMeters(15)),
                new Rotation3d(0, 0, Units.degreesToRadians(90)));
    }
}
