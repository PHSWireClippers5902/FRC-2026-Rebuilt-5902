package org.frc5902.robot.subsystems.SLAMtake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIO;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIOInputsAutoLogged;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIO;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SLAMTake {

    public SlamIO slamIO;
    public IntakeIO intakeIO;

    public SlamIOInputsAutoLogged sIOInputs = new SlamIOInputsAutoLogged();
    public IntakeIOInputsAutoLogged iIOInputs = new IntakeIOInputsAutoLogged();

    private final Alert SlamDisconnectedAlert = new Alert("The SLAM Motor has been disconnected. ", AlertType.kError);
    private final Alert IntakeDisconnectedAlert =
            new Alert("The Intake Motor has been disconnected. ", AlertType.kError);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public SLAMTake(SlamIO sIO, IntakeIO iIO) {
        this.slamIO = sIO;
        this.intakeIO = iIO;
    }

    public void periodic() {
        slamIO.updateInputs(sIOInputs);
        intakeIO.updateInputs(iIOInputs);

        Logger.processInputs("SLAMTake/SlamInputs", sIOInputs);
        Logger.processInputs("SLAMTake/IntakeInputs", iIOInputs);

        SlamDisconnectedAlert.set(!sIOInputs.data.motorConnected());
        IntakeDisconnectedAlert.set(!sIOInputs.data.motorConnected());

        switch (goal) {
            case LOWER_STUPID:
                runVelocities(0, 0.3);
                break;
            case RAISE_STUPID:
                runVelocities(0, -0.3);
                break;
            case RAISED:
                stop();
                break;
            case LOWERED_IDLE:
                stop();
                break;
            case LOWERED_INTAKE:
                runVelocities(35, 0);
                // runSystemVolts(3, 0);
                break;
            case LOWERED_EXTAKE:
                runVelocities(-3, 0);
                break;
            case SHUFFLE:
                stop();
                break;
            case STOP:
                stop();
                break;
            default:
                stop();
                break;
        }
    }

    public enum Goal {
        RAISE_STUPID,
        LOWER_STUPID,
        RAISED,
        LOWERED_IDLE,
        LOWERED_INTAKE,
        LOWERED_EXTAKE,
        SHUFFLE,
        STOP,
    }

    public void runSystemVolts(double intakeVolts, double slamVolts) {
        intakeIO.runVolts(intakeVolts);
        slamIO.runVolts(slamVolts);
    }

    public void runVelocities(double intakeVelocity, double slamVelocity) {
        Logger.recordOutput("Outputs/Intake/IntakeVelocityRotationsPerSecond", intakeVelocity);
        intakeIO.runRadiansPerSecond(intakeVelocity);
        Logger.recordOutput("Outputs/Slam/SlamVelocityRotationsPerSecond", slamVelocity);
        slamIO.runRadiansPerSecond(slamVelocity);
    }

    public void stop() {
        intakeIO.stop();
        slamIO.stop();
    }

    public double getSlamPositionRotations() {
        return sIOInputs.data.positionRotations();
    }
}
