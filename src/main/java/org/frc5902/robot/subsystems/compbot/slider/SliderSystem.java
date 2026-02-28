package org.frc5902.robot.subsystems.compbot.slider;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SliderSystem {
    private final SliderIO sIO;
    private final SliderIOInputsAutoLogged sIOInputs = new SliderIOInputsAutoLogged();
    // SHOULD BE OVEREXAGGERATED
    private final LoggedTunableNumber SLIDER_PREDICTED_LIMIT_STATE = 
        new LoggedTunableNumber("Slider/Slider_PREDICTED_FINAL_LOCATION_OVEREXAGGERATED", 10000);



    private final Alert sliderDisconnectedAlert = new Alert(
            "The SLIDER has been disconnected. IF the SLIDER has deployed, you can still run the intake system"
                    + " normally.",
            AlertType.kWarning);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOW;

    public SliderSystem(SliderIO sIO) {
        this.sIO = sIO;
    }

    @AutoLogOutput
    private boolean firstLimitSwitchActivation = false;
    @AutoLogOutput 
    private double reachedPosition = Double.NaN;
    
    
    public void periodic() {
        sIO.updateInputs(sIOInputs);
        Logger.processInputs("Launcher/Flywheel", sIOInputs);

        sliderDisconnectedAlert.set(sIOInputs.data.motorConnected());
        // TODO IMPLEMENT
        switch (goal) {
            case STOW:
                // run 0 volts. we should NEVER stow in this case until climbing is figured out.
                sIO.runVolts(0.0);
                break;
            case DEPLOYED:
                // if the motor is at its limit STOP...
                if (sIOInputs.data.limitSwitchActivated()) {
                    if (!firstLimitSwitchActivation) {
                        firstLimitSwitchActivation = true;
                        reachedPosition = Rotation2d.fromRadians(sIOInputs.data.positionRads()).getRotations();
                    }
                    sIO.runToPosition(reachedPosition);
                }
                else {
                    if (reachedPosition != Double.NaN){
                        sIO.runToPosition(SLIDER_PREDICTED_LIMIT_STATE.getAsDouble());
                    }
                    else {sIO.runToPosition(reachedPosition);}
                }
                break;
            default:
                sIO.runVolts(0.0);
                break;
        }
    }

    public void runSystemVolts(double insertVolts, double launchVolts) {
        sIO.runVolts(launchVolts);
    }

    public void runVelocitiesVolts(double sliderVelocityPerSecond) {
        Logger.recordOutput("Outputs/Launcher/Flywheel/FlywheelVelocityPerSecond", sliderVelocityPerSecond);
        sIO.runRadiansPerSecond(sliderVelocityPerSecond);
    }

    public void stop() {
        sIO.stop();
    }

    public enum Goal {
        STOW,
        DEPLOYED
    }
}
