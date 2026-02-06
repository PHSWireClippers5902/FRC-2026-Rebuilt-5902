package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.frc5902.robot.Constants.DriveMotorConstants;
import org.frc5902.robot.Constants.SimulatorConstants;
import org.frc5902.robot.Constants.TurnMotorConstants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

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
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SimulatorConstants.driveGearbox,0.025,5.54),
            SimulatorConstants.driveGearbox
        );
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SimulatorConstants.turnGearbox,0.004,41.25),
            SimulatorConstants.turnGearbox
        );

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRadians = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadiansPerSecond = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadiansPerSecond = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRadians = new double[] {inputs.drivePositionRadians};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }


    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond) {
        driveClosedLoop = true;
        driveFFVolts = SimulatorConstants.driveSimKs * Math.signum(velocityRadiansPerSecond) + SimulatorConstants.driveSimKv * velocityRadiansPerSecond;
        driveController.setSetpoint(velocityRadiansPerSecond);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }


}
