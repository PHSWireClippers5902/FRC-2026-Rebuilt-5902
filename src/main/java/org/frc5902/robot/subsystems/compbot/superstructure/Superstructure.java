package org.frc5902.robot.subsystems.compbot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
    private final AgitatorSystem agitator;
    private final IntakeSystem intake;
    private final LauncherSystem launch;
    private final SliderSystem slide;

    @Getter
    @AutoLogOutput
    private OVERALL_GOALS current_state = OVERALL_GOALS.STOW;

    @Getter
    @Setter
    @AutoLogOutput
    private OVERALL_GOALS target_state = OVERALL_GOALS.DEPLOY_IDLE;

    public Superstructure(AgitatorSystem agitator, IntakeSystem intake, LauncherSystem launch, SliderSystem slide) {
        this.agitator = agitator;
        this.intake = intake;
        this.launch = launch;
        this.slide = slide;
    }

    @Override
    public void periodic() {
        // do I manage aim elsewhere? We shall see...
        switch (target_state) {
            case STOW -> {
                launch.setGoal(LauncherSystem.Goal.IDLE);
                slide.setGoal(SliderSystem.Goal.STOW);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
                break;
            }
            case DEPLOY_IDLE -> {
                launch.setGoal(LauncherSystem.Goal.IDLE);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
                break;
            }
            case INTAKE -> {
                // when you are intaking, set slide goal to deployed, agitate the agitator, and intake the intake
                launch.setGoal(LauncherSystem.Goal.IDLE);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);
                intake.setGoal(IntakeSystem.Goal.INTAKE);
                break;
            }
            case INTAKE_PASS -> {
                launch.setGoal(LauncherSystem.Goal.LAUNCH);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_INTAKE);
                intake.setGoal(IntakeSystem.Goal.INTAKE);
                break;
            }
            case OUTTAKE -> {
                launch.setGoal(LauncherSystem.Goal.IDLE);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);
                intake.setGoal(IntakeSystem.Goal.OUTTAKE);
                break;
            }
            case LAUNCH_READY -> {
                launch.setGoal(LauncherSystem.Goal.READY);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.STOP);
                intake.setGoal(IntakeSystem.Goal.STOP);
                break;
            }
            case LAUNCH -> {
                launch.setGoal(LauncherSystem.Goal.LAUNCH);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_INTAKE);
                intake.setGoal(IntakeSystem.Goal.INTAKE_LOW);
                break;
            }
            case CLEAR_JAM_FLYWHEEL -> {
                launch.setGoal(LauncherSystem.Goal.CLEAR_JAM);
                slide.setGoal(SliderSystem.Goal.DEPLOYED);
                agitator.setGoal(AgitatorSystem.Goal.AGITATE_KICK);
                intake.setGoal(IntakeSystem.Goal.INTAKE_LOW);
                break;
            }
        }

        agitator.periodic();
        intake.periodic();
        launch.periodic();
        slide.periodic();
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
