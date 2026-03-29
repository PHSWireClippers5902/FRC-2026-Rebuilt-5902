package org.frc5902.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.launcher.LauncherCalculatorExperimental;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerSystem {
    public IndexerIO indexIO;

    public IndexerIOInputsAutoLogged iIOInputs = new IndexerIOInputsAutoLogged();

    private final Alert IndexerDisconnectedAlert =
            new Alert("The Indexer Motor has been disconnected. ", AlertType.kError);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public IndexerSystem(IndexerIO io) {
        this.indexIO = io;
    }

    public void periodic() {
        indexIO.updateInputs(iIOInputs);

        Logger.processInputs("Indexer/Inputs", iIOInputs);

        IndexerDisconnectedAlert.set(!iIOInputs.data.motorConnected());

        switch (goal) {
            case MOVE_IN:
                runLaunchVelocities(20);
                break;
            case MOVE_OUT:
                runLaunchVelocities(-20);
                break;
            case STOP:
                stop();
                break;
            case SMART_AGITATION:
                if (LauncherCalculatorExperimental.ready()) {
                    runLaunchVelocities(20);
                } else {
                    indexIO.runVolts(0.0);
                }
                break;
            default:
                stop();
                break;
        }
    }

    public void runLaunchVolts(double indexVolts) {
        indexIO.runVolts(indexVolts);
    }

    public void runLaunchVelocities(double rotationsPerSecond) {
        Logger.recordOutput("Outputs/Indexer/IndexVelocity", rotationsPerSecond);
        indexIO.runVelocity(rotationsPerSecond);
    }

    public void stop() {
        indexIO.stop();
    }

    public enum Goal {
        MOVE_IN,
        MOVE_OUT,
        SMART_AGITATION,
        STOP
    }
}
