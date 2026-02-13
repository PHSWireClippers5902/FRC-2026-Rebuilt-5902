package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.frc5902.robot.subsystems.drive.modules.ModuleConfiguration;

public class DriveConstants {
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
}
