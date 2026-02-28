package org.frc5902.robot.subsystems.compbot.slider;

import org.frc5902.robot.util.motorutil.PID;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SliderConstants {
    public static int LimitSwitchPort = -1;
    public static int SliderCANID = -1;
    public static boolean inverted = false;
    public static final double reduction = 1;
    public static int StallLimit = 20;
    public static int FreeLimit = 20;
    public static PID SliderPID =
            PID.builder().proportional(0.1).deriviative(0.1).build();
    public static double SliderPositionConversionFactor = 1.0;
    public static IdleMode idleMode = IdleMode.kBrake;
    
}
