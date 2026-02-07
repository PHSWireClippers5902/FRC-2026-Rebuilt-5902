package org.frc5902.robot.subsystems.kitbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final Alert intakeDisconnected = new Alert("Disconnected Intake.", AlertType.kError);
    private final Alert feederDisconnected = new Alert("Disconnected Feeder.", AlertType.kError);
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.intakeIO = io;
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeIOInputs);
        Logger.processInputs("Intake", intakeIOInputs);

        intakeDisconnected.set(!intakeIOInputs.intakeConnected);
        feederDisconnected.set(!intakeIOInputs.feederConnected);
    }
    /**
     * INTAKE METHODS
     */
    public void intake() {
        this.intake(1, 1);
    }

    public void intake(double IntakePercentOutput, double FeederPercentOutput) {
        intakeIO.setIntakePercentageOutput(Math.abs(IntakePercentOutput));
        intakeIO.setFeederPercentageOutput(Math.abs(FeederPercentOutput));
    }

    public void intake(DoubleSupplier IntakePercentOutput, DoubleSupplier FeederPercentOutput) {
        this.intake(IntakePercentOutput.getAsDouble(),FeederPercentOutput.getAsDouble());
    }
    /**
     * SPIT METHODS
     */
    public void spit() {
        this.spit(1, 1);
    }

    public void spit(double IntakePercentOutput, double FeederPercentOutput) {
        intakeIO.setIntakePercentageOutput(-Math.abs(IntakePercentOutput));
        intakeIO.setFeederPercentageOutput(-Math.abs(FeederPercentOutput));
    }

    public void spit(DoubleSupplier IntakePercentOutput, DoubleSupplier FeederPercentOutput) {
        this.spit(IntakePercentOutput.getAsDouble(),FeederPercentOutput.getAsDouble());
    }
    /**
     * FLYWHEEL METHODS
     */
    public void launch() {
        this.launch(1, 1);
    }

    public void launch(double IntakePercentOutput, double FeederPercentOutput) {
        intakeIO.setIntakePercentageOutput(Math.abs(IntakePercentOutput));
        intakeIO.setFeederPercentageOutput(-Math.abs(FeederPercentOutput));
    }

    public void launch(DoubleSupplier IntakePercentOutput, DoubleSupplier FeederPercentOutput) {
        this.launch(IntakePercentOutput.getAsDouble(),FeederPercentOutput.getAsDouble());
    }

    /**
     * STOP
     */
    public void stop() {
        intakeIO.setIntakePercentageOutput(0);
        intakeIO.setFeederPercentageOutput(0);
    }

    /** AUTOLOG OUTPUT METHODS */
    @AutoLogOutput(key="Intake/IntakePercentOutput")
    public double getIntakePercentOutput(){
        return intakeIOInputs.intakePercentOutput;
    }
    
    @AutoLogOutput(key="Intake/FeederPercentOutput")
    public double getFeederPercentOutput(){
        return intakeIOInputs.feederPercentOutput;
    }
    
}
