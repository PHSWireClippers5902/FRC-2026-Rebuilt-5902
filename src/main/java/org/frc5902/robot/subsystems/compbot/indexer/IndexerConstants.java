package org.frc5902.robot.subsystems.compbot.indexer;

import org.frc5902.robot.util.motorutil.PID;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerConstants {
    public class IndexConstants {
        public static int IndexCANID = -1;
        public static boolean inverted = true;
        public static final double reduction = 1;
        public static int StallLimit = 70;
        public static int FreeLimit = 45;
        public static PID IndexerPID =
                PID.builder().proportional(0.023).deriviative(0.002).build();
        public static double IndexPositionConversionFactor = 1.0;
        public static double IndexVelocityConversionFactor = IndexPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}