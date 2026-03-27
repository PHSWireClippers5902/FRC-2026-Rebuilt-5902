package org.frc5902.robot.subsystems.indexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class IndexerConstants {
    public class IndexConstants {
        public static int IndexCANID = 54;
        public static boolean inverted = false;
        public static final double reduction = 1;
        public static int StallLimit = 10;
        public static int FreeLimit = 20;
        public static PID IndexerPID =
                PID.builder().proportional(0.1).deriviative(0.002).build();
        public static double IndexPositionConversionFactor = 1.0 / 5.0;
        public static double IndexVelocityConversionFactor = IndexPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
