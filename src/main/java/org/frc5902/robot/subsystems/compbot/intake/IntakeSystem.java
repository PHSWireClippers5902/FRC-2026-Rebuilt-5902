package org.frc5902.robot.subsystems.compbot.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

import org.frc5902.robot.subsystems.compbot.intake.intake.*;
import org.frc5902.robot.subsystems.compbot.intake.slider.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSystem extends SubsystemBase {
    private final SliderIO sIO;
    private final IntakeIO iIO;
    private final SliderIOInputsAutoLogged sIOInputs = new SliderIOInputsAutoLogged();
    private final IntakeIOInputsAutoLogged iIOInputs = new IntakeIOInputsAutoLogged();

    private final Alert sliderDisconnectedAlert = new Alert(
            "The SLIDER has been disconnected. IF the SLIDER has deployed, you can still run the intake system"
                    + " normally.",
            AlertType.kWarning);
    private final Alert intakeDisconnectedAlert = new Alert(
            "The INTAKE has been disconnected. Recommended to coordinate with Alliance Partners and swap to defence.",
            AlertType.kError);


    @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOW;



    public IntakeSystem(SliderIO sIO, IntakeIO iIO) {
        this.iIO = iIO;
        this.sIO = sIO;
    }

    @Override
    public void periodic() {
        iIO.updateInputs(iIOInputs);
        sIO.updateInputs(sIOInputs);
        Logger.processInputs("Launcher/Inserter", iIOInputs);
        Logger.processInputs("Launcher/Flywheel", sIOInputs);

        intakeDisconnectedAlert.set(iIOInputs.data.motorConnected());
        sliderDisconnectedAlert.set(sIOInputs.data.motorConnected());
        // TODO IMPLEMENT
        switch (goal) {
            case STOW: ;
            case INTAKE: ;
            case OUTTAKE: ;
            case STOP_INTAKE: ;
            default: ;
        }

    }

    public void runSystemVolts(double insertVolts, double launchVolts) {
        iIO.runVolts(insertVolts);
        sIO.runVolts(launchVolts);
    }

    public void runVelocitiesVolts(double insertVelocityPerSecond, double flywheelVelocityPerSecond) {
        Logger.recordOutput("Outputs/Launcher/Inserter/InsertVelocityPerSecond", insertVelocityPerSecond);
        Logger.recordOutput("Outputs/Launcher/Flywheel/FlywheelVelocityPerSecond", flywheelVelocityPerSecond);
        iIO.runRadiansPerSecond(insertVelocityPerSecond);
        sIO.runRadiansPerSecond(flywheelVelocityPerSecond);
    }

    public void stop() {
        iIO.stop();
        sIO.stop();
    }



    public enum Goal {
        STOW,
        INTAKE,
        OUTTAKE,
        STOP_INTAKE
    }

}
