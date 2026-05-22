package org.frc5902.robot.subsystems.launcher;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.launcher.flywheel.*;
import org.frc5902.robot.subsystems.launcher.inserter.*;
import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LauncherSystem {
    private final InserterIO iIO;
    private final FlywheelIO fIO;
    private final InserterIOInputsAutoLogged iIOInputs = new InserterIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged fIOInputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelEstimation estimation = FlywheelEstimation.getInstance();
    private final LoggedTunableNumber flywheel_velocity = new LoggedTunableNumber("Launcher/Flywheel_Velocity", 190);
    private final LoggedTunableNumber feeder_velocity = new LoggedTunableNumber("Launcher/Feeder_Velocity", 190);

    private final LoggedTunableNumber jam_volts = new LoggedTunableNumber("Launcher/Jam_Volts", -4.0);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.IDLE;

    private final Alert inserterDisconnectedAlert = new Alert(
            "The INSERTER has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);
    private final Alert flywheelLeaderDisconnected = new Alert(
            "The LEFT FLYWHEEL has been disconnected. Recommended to coordinate with Alliance Partners and swap to"
                    + " defence.",
            AlertType.kError);
    private final Alert flywheelFollowerDisconnected = new Alert(
            "The RIGHT FLYWHEEL has been disconnected. Recommended to coordinate with Alliance Partners and swap to"
                    + " defence.",
            AlertType.kError);

    public LauncherSystem(InserterIO iIO, FlywheelIO fIO) {
        this.iIO = iIO;
        this.fIO = fIO;
    }

    public void periodic() {
        iIO.updateInputs(iIOInputs);
        fIO.updateInputs(fIOInputs);
        Logger.processInputs("Launcher/Inserter/Inputs", iIOInputs);
        Logger.processInputs("Launcher/Flywheel/Inputs", fIOInputs);

        inserterDisconnectedAlert.set(iIOInputs.data.motorConnected());
        flywheelLeaderDisconnected.set(fIOInputs.data.leaderConnected());
        flywheelFollowerDisconnected.set(fIOInputs.data.followerConnected());

        switch (goal) {
            case IDLE -> {
                runLaunchVolts(0.0, 0.0);
                break;
            }
            case READY -> {
                // estimation.setGoal(FlywheelEstimation.Goal.HUB);
                // run top motor at flywheel velocity
                // runLaunchVelocities(0, estimation.getTotalFlywheelVelocity());
                double calculatedValue = LauncherCalculatorExperimental.calculate();
                Logger.recordOutput("Outputs/Launcher/CalculateExperimental", calculatedValue);

                fIO.runRotationsPerSecond(calculatedValue);
                break;
            }
            case LAUNCH -> {
                double calculatedValue = LauncherCalculatorExperimental.calculate();
                Logger.recordOutput("Outputs/Launcher/CalculateExperimental", calculatedValue);

                fIO.runRotationsPerSecond(calculatedValue);

                // if (LauncherCalculatorExperimental.ready()) {
                iIO.runRotationsPerSecond(calculatedValue);
                // } else {
                // iIO.runVolts(0.0);
                // }
                break;
            }
            case CLEAR_JAM -> {
                runLaunchVolts(jam_volts.getAsDouble(), jam_volts.getAsDouble());
                break;
            }
            case LAUNCH_STUPID -> {
                runLaunchVelocities(feeder_velocity.getAsDouble(), flywheel_velocity.getAsDouble());
                break;
            }
            case READY_STUPID -> {
                fIO.runRotationsPerSecond(flywheel_velocity.getAsDouble());
                break;
            }
            case SMART_LAUNCH -> {
                double calculatedValue = LauncherCalculatorExperimental.calculate();
                Logger.recordOutput("Outputs/Launcher/CalculateExperimental", calculatedValue);

                fIO.runRotationsPerSecond(calculatedValue);

                if (LauncherCalculatorExperimental.readyWithSpeeds(fIOInputs.data.velocityRPS() * 2, calculatedValue)) {
                    iIO.runRotationsPerSecond(feeder_velocity.getAsDouble());
                } else {
                    iIO.runVolts(0.0);
                }

                break;
            }
            default -> {
                runLaunchVolts(0, 0);
                break;
            }
        }
    }

    // insert, launch
    public double[] launchCalculations() {
        // we would want the top and the botton to be at about equal speeds
        double estimatedTotalVelocity = estimation.getTotalFlywheelVelocity();

        return new double[] {estimatedTotalVelocity, estimatedTotalVelocity};
    }

    public void runLaunchVolts(double insertVolts, double launchVolts) {
        iIO.runVolts(insertVolts);
        fIO.runVolts(launchVolts);
    }

    public void runLaunchVelocities(double insertVelocityPerSecond, double flywheelVelocityPerSecond) {
        Logger.recordOutput("Outputs/Launcher/Inserter/InsertVelocityPerSecond", insertVelocityPerSecond);
        Logger.recordOutput("Outputs/Launcher/Flywheel/FlywheelVelocityPerSecond", flywheelVelocityPerSecond);
        iIO.runRotationsPerSecond(insertVelocityPerSecond);
        fIO.runRotationsPerSecond(flywheelVelocityPerSecond);
    }

    public void stop() {
        iIO.stop();
        fIO.stop();
    }

    public enum Goal {
        IDLE,
        READY,
        LAUNCH,
        SMART_LAUNCH,
        CLEAR_JAM,
        READY_STUPID,
        LAUNCH_STUPID
    }
}
