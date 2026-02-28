package org.frc5902.robot.subsystems.compbot.agitator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.util.buildutil.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AgitatorSystem extends SubsystemBase {
    private static final LoggedTunableNumber agitatorAgitateVolts =
            new LoggedTunableNumber("Agitator/AgitateVolts", 4.0);
    private static final LoggedTunableNumber agitatorKickVolts = new LoggedTunableNumber("Agitator/AgitateVolts", -4.0);

    private final AgitatorIO io;
    private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();
    private final Alert agitatorConnectedAlert = new Alert(
            "The AGITATOR has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);

    @Getter
    @Setter
    @AutoLogOutput
    private Goal goal = Goal.STOP;

    public AgitatorSystem(AgitatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Agitator/inputs", inputs);
        agitatorConnectedAlert.set(inputs.data.motorConnected());

        switch (goal) {
            case AGITATE_INTAKE:
                runVolts(agitatorAgitateVolts.getAsDouble());
                break;
            case AGITATE_KICK:
                runVolts(agitatorKickVolts.getAsDouble());
                break;
            default:
                runVolts(0.0);
                break;
        }
    }

    public void stop() {
        io.stop();
    }

    @AutoLogOutput
    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public enum Goal {
        AGITATE_INTAKE,
        AGITATE_KICK,
        STOP
    }
}
