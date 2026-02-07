package org.frc5902.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5902.robot.subsystems.kitbot.intake.Intake;

import java.util.function.DoubleSupplier;

public class IntakeCommands {
    private IntakeCommands() {}

    public static Command intakeDefaultCommand(Intake intake) {
        return Commands.run(
                // Run
                () -> {
                    intake.intake();
                },
                // Requirements
                intake);
    }

    public static Command intakeCommand(
            Intake intake, DoubleSupplier intakePercentOutput, DoubleSupplier feederPercentOutput) {
        return Commands.run(
                // Run
                () -> {
                    intake.intake(intakePercentOutput, feederPercentOutput);
                },
                // Requirements
                intake);
    }

    public static Command spit(Intake intake) {
        return Commands.run(
                // Run
                () -> {
                    intake.spit();
                },
                // Requirements
                intake);
    }

    public static Command flywheelDefaultCommand(Intake intake) {
        return Commands.run(
                // Run
                () -> {
                    intake.launch();
                },
                // Requirements
                intake);
    }

    public static Command flywheelCommand(
            Intake intake, DoubleSupplier intakePercentOutput, DoubleSupplier feederPercentOutput) {
        return Commands.run(
                // Run
                () -> {
                    intake.launch(intakePercentOutput, feederPercentOutput);
                },
                // Requirements
                intake);
    }

    public static Command stopCommand(Intake intake) {
        return Commands.runOnce(
                () -> {
                    intake.stop();
                },
                intake);
    }
}
