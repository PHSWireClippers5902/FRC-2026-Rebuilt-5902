package org.frc5902.robot.subsystems.SLAMtake.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class IntakeSystemConstants {
    public class IntakeConstants {
        public static int IntakeCANID = 53;
        public static boolean inverted = false;
        public static final double reduction = 1;
        public static int StallLimit = 70;
        public static int FreeLimit = 45;
        public static PID IntakePID =
                PID.builder().proportional(0.3).deriviative(0.002).build();
        public static double IntakePositionConversionFactor = 1.0 / 3.0;
        public static double IntakeVelocityConversionFactor = IntakePositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
