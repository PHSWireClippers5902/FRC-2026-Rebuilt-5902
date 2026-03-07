package org.frc5902.robot.subsystems.compbot.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.frc5902.robot.subsystems.compbot.superstructure.SuperstructureActions.SuperstructureAction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class Superstructure extends SubsystemBase {
    @Getter
    private final AgitatorSystem agitator;

    @Getter
    private final IntakeSystem intake;

    @Getter
    private final LauncherSystem launch;

    @Getter
    private final SliderSystem slide;

    @Getter
    private static Superstructure instance = null;

    @Getter
    @Setter
    @AutoLogOutput
    private boolean KILL_SYSTEMS = true;

    @Getter
    private SuperstructureAction goal = SuperstructureActions.DEPLOY_IDLE;

    private ArrayList<SuperstructureAction> scheduled_goals = new ArrayList<SuperstructureAction>();

    public Superstructure(AgitatorSystem agitator, IntakeSystem intake, LauncherSystem launch, SliderSystem slide) {
        // schedule the default command, deploy & idle
        this.agitator = agitator;
        this.intake = intake;
        this.launch = launch;
        this.slide = slide;
        SuperstructureAction.setStaticSubsystems(launch, agitator, slide, intake);
        scheduled_goals.add(SuperstructureActions.DEPLOY_IDLE);
        instance = this;
    }

    @Override
    public void periodic() {
        prioritizeGoals();
        if (KILL_SYSTEMS) {
            goal = SuperstructureActions.EMERGENCY;
        }
        goal.set();
        Logger.recordOutput("Superstructure/Goal", goal.toString());
    }

    public void prioritizeGoals() {
        // default goal
        SuperstructureAction maximumGoal = SuperstructureActions.STOW;
        for (SuperstructureAction g : scheduled_goals) {
            if (g.getPriority() >= maximumGoal.getPriority()) {
                maximumGoal = g;
            }
        }
        this.goal = maximumGoal;
    }

    public InstantCommand addCommandToScheduler(SuperstructureAction g) {
        return new InstantCommand(() -> scheduled_goals.add(g), this);
    }

    public InstantCommand removeCommandFromScheduler(SuperstructureAction g) {
        return new InstantCommand(() -> scheduled_goals.remove(g), this);
    }

    public InstantCommand SWAP_KILL_SYSTEMS() {
        return new InstantCommand(() -> this.KILL_SYSTEMS = !this.KILL_SYSTEMS);
    }
}
