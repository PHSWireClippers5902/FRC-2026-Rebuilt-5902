package org.frc5902.robot.subsystems.compbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSystem extends SubsystemBase {
    private final IntakeIO iIO;
    private final IntakeIOInputsAutoLogged iIOInputs = new IntakeIOInputsAutoLogged();

    private final Alert intakeDisconnectedAlert = new Alert(
            "The INTAKE has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public IntakeSystem(IntakeIO iIO) {
        this.iIO = iIO;
    }

    @Override
    public void periodic() {
        iIO.updateInputs(iIOInputs);
        Logger.processInputs("Launcher/Inserter", iIOInputs);

        intakeDisconnectedAlert.set(iIOInputs.data.motorConnected());
        // TODO IMPLEMENT
        switch (goal) {
            case INTAKE:
                ;
            case OUTTAKE:
                ;
            case STOP:
                ;
            default:
                ;
        }
    }

    public void runSystemVolts(double insertVolts) {
        iIO.runVolts(insertVolts);
    }

    public void runVelocitiesVolts(double insertVelocityPerSecond) {
        Logger.recordOutput("Outputs/Launcher/Inserter/InsertVelocityPerSecond", insertVelocityPerSecond);
        iIO.runRadiansPerSecond(insertVelocityPerSecond);
    }

    public void stop() {
        iIO.stop();
    }

    public enum Goal {
        INTAKE,
        OUTTAKE,
        STOP
    }
}
