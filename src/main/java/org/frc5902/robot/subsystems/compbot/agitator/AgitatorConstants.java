package org.frc5902.robot.subsystems.compbot.agitator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.PID;

public class AgitatorConstants {
    public static int AgitatorCANID = -1;
    public static boolean inverted = false;
    public static final double reduction = 1;
    public static int StallLimit = 20;
    public static int FreeLimit = 20;
    public static PID agitatorPID =
            PID.builder().proportional(0.1).deriviative(0.1).build();
    public static double agitatorPositionConversionFactor = 1.0;

    public static IdleMode idleMode = IdleMode.kCoast;
}
