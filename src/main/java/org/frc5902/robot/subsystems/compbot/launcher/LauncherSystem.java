package org.frc5902.robot.subsystems.compbot.launcher;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

import org.frc5902.robot.subsystems.compbot.launcher.flywheel.*;
import org.frc5902.robot.subsystems.compbot.launcher.inserter.*;
import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class LauncherSystem {
    private final InserterIO iIO;
    private final FlywheelIO fIO;
    private final InserterIOInputsAutoLogged iIOInputs = new InserterIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged fIOInputs = new FlywheelIOInputsAutoLogged();
    private final FlywheelEstimation estimation = FlywheelEstimation.getInstance();

    private final LoggedTunableNumber jam_volts = new LoggedTunableNumber("Launcher/Jam_Volts", -4.0);
    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.IDLE;
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

    
    public void periodic() {
        iIO.updateInputs(iIOInputs);
        fIO.updateInputs(fIOInputs);
        Logger.processInputs("Launcher/Inserter", iIOInputs);
        Logger.processInputs("Launcher/Flywheel", fIOInputs);

        inserterDisconnectedAlert.set(iIOInputs.data.motorConnected());
        flywheelDisconnectedAlert.set(fIOInputs.data.motorConnected());

        switch (goal) {
            case IDLE -> {
                break;
            }
            case READY -> {
                
                // run top motor at flywheel velocity
                runLaunchVelocities(0, estimation.getTotalFlywheelVelocity());
                break;
            }
            case LAUNCH -> {
                estimation.setGoal(FlywheelEstimation.Goal.HUB);
                runLaunchVelocities(launchCalculations()[0], launchCalculations()[1]);
                break;
            }
            case CLEAR_JAM -> {
                runLaunchVolts(jam_volts.getAsDouble(), jam_volts.getAsDouble());
                break;
            }
            default -> {
                runLaunchVolts(0,0);
                break;
            }
            
        }

    }

    // insert, launch
    public double[] launchCalculations() {
        // we would want the top and the botton to be at about equal speeds
        double estimatedTotalVelocity = estimation.getTotalFlywheelVelocity();

        return new double[]{estimatedTotalVelocity/2,estimatedTotalVelocity/2};
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

    public enum Goal {
        IDLE,
        READY,
        LAUNCH,
        CLEAR_JAM
        
    }


}
