package org.frc5902.robot.subsystems.compbot.slider;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
            new LoggedTunableNumber("Slider/SLIDER_OVEREXXAGERATED", 4);

    private final Alert sliderDisconnectedAlert = new Alert(
            "The SLIDER has been disconnected. IF the SLIDER has deployed, you can still run the intake system"
                    + " normally.",
            AlertType.kWarning);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOW;

    @Getter
    @Setter
    @AutoLogOutput
    private boolean EMERGENCY_OVERRIDE = false;

    @Getter
    @Setter
    @AutoLogOutput
    private State state = State.STOWED;

    public SliderSystem(SliderIO sIO) {
        this.sIO = sIO;
    }

    @AutoLogOutput
    private boolean firstLimitSwitchActivation = false;

    @AutoLogOutput
    private double reachedPosition = Double.NaN;

    public void periodic() {
        sIO.updateInputs(sIOInputs);
        Logger.processInputs("Slider/Inputs", sIOInputs);
        sliderDisconnectedAlert.set(sIOInputs.data.motorConnected());
        switch (goal) {
            case STOW:
                // run 0 volts. we should NEVER stow in this case until climbing is figured out.
                sIO.runVolts(0.0);
                break;
            case DEPLOYED:
                // if the motor is at its limit STOP...
                if (sIOInputs.data.limitSwitchActivated()) {
                    // completely deployed now
                    this.state = SliderSystem.State.DEPLOYED;
                    sIO.runVolts(0);
                } else {
                    sIO.runVolts(SLIDER_PREDICTED_LIMIT_STATE.getAsDouble());
                }
                break;
            case STOP:
                sIO.runVolts(0);
                break;
            default:
                sIO.runVolts(0.0);
                break;
        }
    }

    public void runSystemVolts(double insertVolts, double launchVolts) {
        sIO.runVolts(launchVolts);
    }

    public void runVelocitiesVolts(double sliderRotationsPerSecond) {
        Logger.recordOutput("Outputs/Slider/SliderRotationsPerSecond", sliderRotationsPerSecond);
        sIO.runRadiansPerSecond(sliderRotationsPerSecond);
    }

    public void stop() {
        sIO.stop();
    }

    public enum Goal {
        STOW,
        DEPLOYED,
        STOP
    }

    public enum State {
        STOWED,
        DEPLOYED
    }
}
