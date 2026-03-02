package org.frc5902.robot.subsystems.compbot.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class IntakeSystemConstants {
    public class IntakeConstants {
        public static int IntakeCANID = 40;
        public static boolean inverted = true;
        public static final double reduction = 1;
        public static int StallLimit = 20;
        public static int FreeLimit = 20;
        public static PID IntakePID =
                PID.builder().proportional(0.1).deriviative(0.1).build();
        public static double IntakePositionConversionFactor = 1.0;

        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
