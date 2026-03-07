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
    private final LoggedTunableNumber deployVolts = new LoggedTunableNumber("Intake/Tuneable/DeployVolts", 7.0);
    private final LoggedTunableNumber intakeRPS = new LoggedTunableNumber("Intake/Tuneable/IntakeRPS", 90);
    private final LoggedTunableNumber intakeLowRPS = new LoggedTunableNumber("Intake/Tuneable/IntakeLowRPS", 12);
    private final LoggedTunableNumber outtakeRPS = new LoggedTunableNumber("Intake/Tuneable/outtakeRPS", -30);
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
        switch (goal) {
            case INTAKE:
                // iIO.runVolts(intakeVolts.getAsDouble());
                iIO.runRadiansPerSecond(intakeRPS.getAsDouble());
                break;
            case INTAKE_LOW:
                // iIO.runVolts(intakeLowVolts.getAsDouble());
                // iIO.runRadiansPerSecond(intakeLowRPS.getAsDouble());
                // for now run nothing
                iIO.runRadiansPerSecond(intakeRPS.getAsDouble() * 0.2);
                break;
            case OUTTAKE:
                // iIO.runVolts(outtakeVolts.getAsDouble());
                iIO.runRadiansPerSecond(outtakeRPS.getAsDouble());
                break;
            case STOP:
                iIO.runVolts(0);
                break;
            case DEPLOY:
                if (Superstructure.getInstance() != null
                        && Superstructure.getInstance().getSlide().getState() != SliderSystem.State.DEPLOYED) {
                    iIO.runVolts(deployVolts.getAsDouble()); // switch to rps?
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
