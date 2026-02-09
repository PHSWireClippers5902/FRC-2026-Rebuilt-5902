package org.frc5902.robot.subsystems.kitbot.intake;

import com.ctre.phoenix.motorcontrol.InvertType;

public class IntakeConstants {
    public static final int FeederCANID = 40;
    public static final int IntakeCANID = 41;

    // is the feeder / intake inverted? We want anything intake to be positive
    public static final InvertType FeederInverted = InvertType.None;
    public static final InvertType IntakeInverted = InvertType.None;
}
