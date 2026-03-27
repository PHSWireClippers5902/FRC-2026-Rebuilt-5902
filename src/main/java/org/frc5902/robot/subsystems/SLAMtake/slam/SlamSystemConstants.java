package org.frc5902.robot.subsystems.SLAMtake.slam;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class SlamSystemConstants {
    public class SlamConstants {
        public static int SlamCANID = 52;
        public static boolean inverted = false;
        public static final double reduction = 1;
        public static int StallLimit = 45;
        public static int FreeLimit = 30;
        public static PID SlamPID =
                PID.builder().proportional(2.3).deriviative(0.002).build();
        public static double SlamPositionConversionFactor = 0.01;
        public static double SlamVelocityConversionFactor = SlamPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;

        public static int SlamLimitSwitchID = 9;
        // public static boolean SlamLimitSwitchOnByDefault = true;
        public static double EstimatedTopValue = 0.21;
        public static double raisedAngle = 0.09;
    }
}
