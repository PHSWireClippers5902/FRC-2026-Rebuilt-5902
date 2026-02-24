package org.frc5902.robot.subsystems.compbot.launcher;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5902.robot.subsystems.compbot.launcher.flywheel.*;
import org.frc5902.robot.subsystems.compbot.launcher.inserter.*;
import org.littletonrobotics.junction.Logger;

public class LauncherSystem extends SubsystemBase {
    private final InserterIO iIO;
    private final FlywheelIO fIO;
    private final InserterIOInputsAutoLogged iIOInputs = new InserterIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged fIOInputs = new FlywheelIOInputsAutoLogged();

    private final Alert inserterDisconnectedAlert = new Alert(
            "The INSERTER has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);
    private final Alert flywheelDisconnectedAlert = new Alert(
            "The FLYWHEEL has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);

    public LauncherSystem(InserterIO iIO, FlywheelIO fIO) {
        this.iIO = iIO;
        this.fIO = fIO;
    }

    @Override
    public void periodic() {
        iIO.updateInputs(iIOInputs);
        fIO.updateInputs(fIOInputs);
        Logger.processInputs("Launcher/Inserter", iIOInputs);
        Logger.processInputs("Launcher/Flywheel", fIOInputs);

        inserterDisconnectedAlert.set(iIOInputs.data.motorConnected());
        flywheelDisconnectedAlert.set(fIOInputs.data.motorConnected());
    }

    public void runLaunchVolts(double insertVolts, double launchVolts) {
        iIO.runVolts(insertVolts);
        fIO.runVolts(launchVolts);
    }

    public void runLaunchVelocities(double insertVelocityPerSecond, double flywheelVelocityPerSecond) {
        Logger.recordOutput("Outputs/Launcher/Inserter/InsertVelocityPerSecond", insertVelocityPerSecond);
        Logger.recordOutput("Outputs/Launcher/Flywheel/FlywheelVelocityPerSecond", flywheelVelocityPerSecond);
        iIO.runRadiansPerSecond(insertVelocityPerSecond);
        fIO.runRadiansPerSecond(flywheelVelocityPerSecond);
    }

    public void stop() {
        iIO.stop();
        fIO.stop();
    }
}
