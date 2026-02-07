package org.frc5902.robot.subsystems.kitbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        intakeIO.setIntakePercentageOutput(IntakePercentOutput);
        intakeIO.setFeederPercentageOutput(FeederPercentOutput);
    }

    public void intake(DoubleSupplier IntakePercentOutput, DoubleSupplier FeederPercentOutput) {
        this.intake(IntakePercentOutput.getAsDouble(), FeederPercentOutput.getAsDouble());
    }

    /**
     *
     */
    public void stop() {
        intakeIO.setIntakePercentageOutput(0);
        intakeIO.setFeederPercentageOutput(0);
    }
}
