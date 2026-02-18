package org.frc5902.robot.subsystems.compbot.agitator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSystem extends SubsystemBase{
    private final AgitatorIO io;
    private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged(); 
    private final Alert agitatorConnectedAlert = new Alert(
            "The AGITATOR has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);

    public AgitatorSystem(AgitatorIO io){
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Agitator", inputs);
        agitatorConnectedAlert.set(inputs.data.motorConnected());


    }

    public void stop() {
        io.stop();
    }

    @AutoLogOutput
    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    @AutoLogOutput
    public void runVelocity(double velocityRadiansPerSecond){
        io.runRadiansPerSecond(velocityRadiansPerSecond);
    }

}
