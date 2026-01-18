package org.frc5902.robot.subsystems.drive;



import org.frc5902.robot.Constants.*;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        
        driveDisconnectedAlert = new Alert(
            "Disconnected drive motor on module " + index + ".",
            AlertType.kError
        );
        turnDisconnectedAlert = new Alert(
            "Disconnected turn motor on module " + index + ".",
            AlertType.kError
        );
    }


    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
        
        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++){
            double positionMeters = inputs.odometryDrivePositionsRadians[i] * DriveMotorConstants.driveWheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // alert 
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);

    }

    // mutate and modify swerve module state
    public void runSetpoint(SwerveModuleState state){
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // set
        io.setDriveVelocity(state.speedMetersPerSecond / DriveMotorConstants.driveWheelRadius);
        io.setTurnPosition(state.angle);
    }

    // runs module with output while having zero degrees as turn position
    public void runCharacterization(double output){
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    // disables motor
    public void stop(){
        io.setDriveOpenLoop(0);
        io.setTurnOpenLoop(0);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * DriveMotorConstants.driveWheelRadius;
    }

    public double getVelocityMetersPerSecond() {
        return inputs.driveVelocityRadiansPerSecond * DriveMotorConstants.driveWheelRadius;
    }

    // position, angle
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    // velocity, angle
    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
    }

    // updated in periodic()
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharactierizationPosition() {
        return inputs.drivePositionRadians;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadiansPerSecond;
    }

}
