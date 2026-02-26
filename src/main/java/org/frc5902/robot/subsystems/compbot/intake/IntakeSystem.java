package org.frc5902.robot.subsystems.compbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSystem extends SubsystemBase {
    private final IntakeIO iIO;
    private final IntakeIOInputsAutoLogged iIOInputs = new IntakeIOInputsAutoLogged();
    

    private final LoggedTunableNumber intakeVolts = new LoggedTunableNumber("Intake/Tuneable/IntakeVolts", 12.0);
    private final LoggedTunableNumber intakeLowVolts = new LoggedTunableNumber("Intake/Tuneable/IntakeVolts", 4.0);
    private final LoggedTunableNumber outtakeVolts = new LoggedTunableNumber("Intake/Tuneable/IntakeVolts", -12.0);



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
                iIO.runVolts(intakeVolts.getAsDouble());
            case INTAKE_LOW: 
                iIO.runVolts(intakeLowVolts.getAsDouble());
            case OUTTAKE:
                iIO.runVolts(outtakeVolts.getAsDouble());
            case STOP:
                iIO.runVolts(0);
            default:
                iIO.runVolts(0);
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
        INTAKE_LOW,
        OUTTAKE,
        STOP
    }
}
