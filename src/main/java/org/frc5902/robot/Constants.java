/**
 * @team: The Wire Clippers 5902
 * @name:    Constants.java
 * @purpose: A single place to store program-wide constants. Updated for 2026.
 * @name:    Daniel Sabalakov
 */
package org.frc5902.robot;
// import frc.robot.helpers.Gains;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final class SwervePIDConstants {
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = true;

        // public static final Gains kGains = new Gains(0.5, 0.0000, 1, 0.0, 0, 1.0);

        // public static final double drivekP = ;
        // public static final double drivekI = ;

    }

    public static final class Translations {
        // public static final double xPos = 0;
        // public static final double yPos = 0;
    }

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

    public static final class RobotInitializationConstants {
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
    }
}
