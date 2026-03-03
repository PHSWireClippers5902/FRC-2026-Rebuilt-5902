package org.frc5902.robot.subsystems.compbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.frc5902.robot.subsystems.compbot.superstructure.Superstructure;
import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSystem {
    private final IntakeIO iIO;
    private final IntakeIOInputsAutoLogged iIOInputs = new IntakeIOInputsAutoLogged();

    private final LoggedTunableNumber intakeVolts = new LoggedTunableNumber("Intake/Tuneable/IntakeVolts", 3.0);
    private final LoggedTunableNumber intakeLowVolts = new LoggedTunableNumber("Intake/Tuneable/IntakeLowVolts", 2.0);
    private final LoggedTunableNumber deployVolts = new LoggedTunableNumber("Intake/Tuneable/DeployVolts", 9.0);

    private final LoggedTunableNumber outtakeVolts = new LoggedTunableNumber("Intake/Tuneable/OuttakeVolts", -3.0);

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

    public void periodic() {
        iIO.updateInputs(iIOInputs);
        Logger.processInputs("Intake/Inputs", iIOInputs);

        intakeDisconnectedAlert.set(iIOInputs.data.motorConnected());
        // TODO IMPLEMENT
        switch (goal) {
            case INTAKE:
                // iIO.runVolts(intakeVolts.getAsDouble());
                iIO.runRadiansPerSecond(300);
                break;
            case INTAKE_LOW:
                // iIO.runVolts(intakeLowVolts.getAsDouble());
                iIO.runRadiansPerSecond(100);

                break;
            case OUTTAKE:
                // iIO.runVolts(outtakeVolts.getAsDouble());
                iIO.runRadiansPerSecond(300);
                break;
            case STOP:
                iIO.runVolts(0);
                break;
            case DEPLOY:
                if (Superstructure.getInstance() != null
                        && Superstructure.getInstance().getSlide().getState() != SliderSystem.State.DEPLOYED) {
                    iIO.runVolts(deployVolts.getAsDouble());
                } else {
                    iIO.runVolts(0);
                }
                break;
            default:
                iIO.runVolts(0);
                break;
        }
    }

    public void runSystemVolts(double insertVolts) {
        iIO.runVolts(insertVolts);
    }

    public void runVelocitiesVolts(double insertVelocityPerSecond) {
        Logger.recordOutput("Outputs/Intake/IntakeVelocityRotationsPerSecond", insertVelocityPerSecond);
        iIO.runRadiansPerSecond(insertVelocityPerSecond);
    }

    public void stop() {
        iIO.stop();
    }

    public enum Goal {
        INTAKE,
        INTAKE_LOW,
        OUTTAKE,
        STOP,
        DEPLOY,
    }
}
