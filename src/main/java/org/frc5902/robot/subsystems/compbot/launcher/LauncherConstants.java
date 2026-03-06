package org.frc5902.robot.subsystems.compbot.launcher;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class LauncherConstants {
    public class FlywheelConstants {
        public static int FlywheelCANID = 50;
        public static boolean inverted = true;
        public static final double reduction = 1;
        public static int StallLimit = 70;
        public static int FreeLimit = 45;
        public static PID flywheelPID =
                PID.builder().proportional(0.3).deriviative(0.1).build();
        public static double flywheelPositionConversionFactor = 1.0;
        public static double flywheelVelocityConversionFactor = flywheelPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }

    public class InserterConstants {
        public static int InserterCANID = 51;
        public static boolean inverted = false;
        public static final double reduction = 1;
        public static int StallLimit = 70; // can go with less bcz doesnt need same speeds - BigD
        public static int FreeLimit = 45;
        public static PID inserterPID =
                PID.builder().proportional(0.3).deriviative(0.1).build();
        public static double inserterPositionConversionFactor = 1.0;
        public static double inserterVelocityConversionFactor = inserterPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
