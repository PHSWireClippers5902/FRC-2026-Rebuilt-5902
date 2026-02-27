package org.frc5902.robot.subsystems.compbot.superstructure;

import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.frc5902.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends SubsystemBase {
    private final AgitatorSystem agitator;
    private final IntakeSystem intake;
    private final LauncherSystem launch;
    private final SliderSystem slide;
    @Getter @AutoLogOutput
    private OVERALL_GOALS current_state = OVERALL_GOALS.STOW;
    @Getter @Setter @AutoLogOutput
    private OVERALL_GOALS target_state = OVERALL_GOALS.DEPLOY_IDLE;


    public Superstructure(
        AgitatorSystem agitator,
        IntakeSystem intake,
        LauncherSystem launch,
        SliderSystem slide
    ) {
        this.agitator = agitator;
        this.intake = intake;
        this.launch = launch;
        this.slide = slide;
    }

    @Override
    public void periodic() {
        switch (target_state) {
            case STOW -> {
                slide.setGoal(SliderSystem.Goal.STOW);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
            }
            case DEPLOY_IDLE -> {
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
            }
            case INTAKE -> {
                // when you are intaking, set slide goal to deployed, agitate the agitator, and intake the intake
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);
                intake.setGoal(IntakeSystem.Goal.INTAKE);
            }
            case INTAKE_PASS -> {
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_INTAKE);
                intake.setGoal(IntakeSystem.Goal.INTAKE);
            }
            case OUTTAKE -> {
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);
                intake.setGoal(IntakeSystem.Goal.OUTTAKE);
            }
            case LAUNCH_READY -> {
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
            }
            case LAUNCH -> {
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_INTAKE);
                intake.setGoal(IntakeSystem.Goal.INTAKE_LOW);
            }
            case CLEAR_JAM_FLYWHEEL -> {                
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);     
                intake.setGoal(IntakeSystem.Goal.INTAKE_LOW);
            }
        }
    }
    

    public enum OVERALL_GOALS {
        STOW,
        DEPLOY_IDLE,
        INTAKE,
        INTAKE_PASS,
        OUTTAKE,
        LAUNCH_READY,
        LAUNCH,
        CLEAR_JAM_FLYWHEEL,
    }


}
