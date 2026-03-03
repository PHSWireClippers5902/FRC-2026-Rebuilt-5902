package org.frc5902.robot.subsystems.compbot.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.frc5902.robot.subsystems.compbot.superstructure.SuperstructureActions.SuperstructureAction;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.ArrayList;

public class Superstructure extends SubsystemBase {
    @Getter
    @AutoLogOutput
    private SuperstructureAction goal = SuperstructureActions.DEPLOY_IDLE;

    private ArrayList<SuperstructureAction> scheduled_goals = new ArrayList<SuperstructureAction>();

    public Superstructure(AgitatorSystem agitator, IntakeSystem intake, LauncherSystem launch, SliderSystem slide) {
        // schedule the default command, deploy & idle
        SuperstructureAction.setStaticSubsystems(launch,agitator,slide,intake);
    }

    @Override
    public void periodic() {
        prioritizeGoals();
        goal.set();
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

    
}
