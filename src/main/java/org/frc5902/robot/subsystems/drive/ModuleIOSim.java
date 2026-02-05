package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import org.frc5902.robot.Constants.DriveMotorConstants;
import org.frc5902.robot.Constants.TurnMotorConstants;

public class ModuleIOSim implements ModuleIO {
    // private final DCMotorSim driveSim;
    // private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(
            DriveMotorConstants.driveClosedLoop.getProportional(),
            DriveMotorConstants.driveClosedLoop.getIntegral(),
            DriveMotorConstants.driveClosedLoop.getDeriviative());
    private PIDController turnController = new PIDController(
            TurnMotorConstants.turnClosedLoop.getProportional(),
            TurnMotorConstants.turnClosedLoop.getIntegral(),
            TurnMotorConstants.turnClosedLoop.getDeriviative());
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        // driveSim = new DCMotorSim(
        //     LinearSystemId.createDCMotorSystem(, )
        // )
    }
}
