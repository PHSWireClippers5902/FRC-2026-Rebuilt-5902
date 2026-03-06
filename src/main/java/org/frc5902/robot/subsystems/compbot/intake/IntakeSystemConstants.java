package org.frc5902.robot.subsystems.compbot.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class IntakeSystemConstants {
    public class IntakeConstants {
        public static int IntakeCANID = 40;
        public static boolean inverted = true;
        public static final double reduction = 1;
        public static int StallLimit = 70;
        public static int FreeLimit = 45;
        public static PID IntakePID =
                PID.builder().proportional(0.035).deriviative(0.002).build();
        public static double IntakePositionConversionFactor = 1.0;
        public static double IntakeVelocityConversionFactor = IntakePositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
