package org.frc5902.robot.subsystems.compbot.launcher;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class LauncherConstants {
    public class FlywheelConstants {
        public static int FlywheelCANID = -1;
        public static boolean inverted = false;    
        public static final double reduction = 1;
        public static int StallLimit = 20;
        public static int FreeLimit = 20;
        public static IdleMode idleMode = IdleMode.kBrake;

    }
    public class InserterConstants {
        public static int InserterCANID = -1;
        public static boolean inverted = false;
        public static final double reduction = 1;
    }
}   
