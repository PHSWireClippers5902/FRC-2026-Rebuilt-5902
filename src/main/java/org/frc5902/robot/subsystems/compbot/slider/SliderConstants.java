package org.frc5902.robot.subsystems.compbot.slider;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.frc5902.robot.util.motorutil.PID;

public class SliderConstants {
    public static int LimitSwitchPort = 0;
    public static int SliderCANID = 53;
    public static boolean inverted = true;
    public static final double reduction = 1;
    public static int StallLimit = 20;
    public static int FreeLimit = 20;
    public static PID SliderPID =
            PID.builder().proportional(0.03).deriviative(0.1).build();
    public static double SliderPositionConversionFactor = 1.0;
    public static IdleMode idleMode = IdleMode.kBrake;
}
