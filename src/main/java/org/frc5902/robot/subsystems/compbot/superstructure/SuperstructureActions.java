package org.frc5902.robot.subsystems.compbot.superstructure;

import lombok.Builder;
import lombok.Getter;
import lombok.ToString;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;

public class SuperstructureActions {

    // STOW: Maintain defaults BUT have STOW
    public static SuperstructureAction STOW = SuperstructureAction.builder()
            .sliderGoal(SliderSystem.Goal.STOW)
            .priority(0)
            .build();
    // DEPLOY_IDLE: Maintain all defaults
    public static SuperstructureAction DEPLOY_IDLE = SuperstructureAction.builder()
            .intakeGoal(IntakeSystem.Goal.DEPLOY)
            .priority(0)
            .build();
    // INTAKE: Run Intake and Agitate Out
    public static SuperstructureAction INTAKE = SuperstructureAction.builder()
            .intakeGoal(IntakeSystem.Goal.INTAKE)
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_KICK)
            .priority(1)
            .build();
    // INTAKE AND PASS: Run intake, flywheel, and agitator in
    public static SuperstructureAction INTAKE_AND_PASS = SuperstructureAction.builder()
            .intakeGoal(IntakeSystem.Goal.INTAKE)
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_INTAKE)
            .launcherGoal(LauncherSystem.Goal.LAUNCH)
            .priority(2)
            .build();
    // OUTTAKE: Clear by kicking agitator AND running Outtake
    public static SuperstructureAction OUTTAKE = SuperstructureAction.builder()
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_KICK)
            .intakeGoal(IntakeSystem.Goal.OUTTAKE)
            .priority(1)
            .build();
    // READY_LAUNCHER: Runs launcher at ready state.
    public static SuperstructureAction READY_LAUNCHER = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.READY)
            .priority(2)
            .build();
    // LAUNCH: Run agitate in, intake at low to keep fuel in, and launch system
    public static SuperstructureAction LAUNCH = SuperstructureAction.builder()
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_INTAKE)
            .intakeGoal(IntakeSystem.Goal.INTAKE_LOW)
            .launcherGoal(LauncherSystem.Goal.LAUNCH)
            .priority(3)
            .build();
    // CLEAR_FLYWHEEL_JAM: Run agitator to kick fuel out, run intake in, and run launcher system to clear jam
    public static SuperstructureAction CLEAR_FLYWHEEL_JAM = SuperstructureAction.builder()
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_KICK)
            .intakeGoal(IntakeSystem.Goal.INTAKE_LOW)
            .launcherGoal(LauncherSystem.Goal.CLEAR_JAM)
            .priority(4)
            .build();

    // READY_LAUNCHER: Runs launcher at ready state.
    public static SuperstructureAction READY_LAUNCHER_STUPID = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.READY_STUPID)
            .priority(2)
            .build();
    // LAUNCH: Run agitate in, intake at low to keep fuel in, and launch system
    public static SuperstructureAction LAUNCH_STUPID = SuperstructureAction.builder()
            .agitatorGoal(AgitatorSystem.Goal.AGITATE_INTAKE)
            .intakeGoal(IntakeSystem.Goal.INTAKE_LOW)
            .launcherGoal(LauncherSystem.Goal.LAUNCH_STUPID)
            .priority(3)
            .build();

    public static SuperstructureAction EMERGENCY = SuperstructureAction.builder()
            .agitatorGoal(AgitatorSystem.Goal.STOP)
            .intakeGoal(IntakeSystem.Goal.STOP)
            .launcherGoal(LauncherSystem.Goal.IDLE)
            .sliderGoal(SliderSystem.Goal.STOP)
            .build();

    @Builder
    @ToString
    public static class SuperstructureAction {
        // Defaults for all actions
        @Builder.Default
        private LauncherSystem.Goal launcherGoal = LauncherSystem.Goal.IDLE;

        @Builder.Default
        private AgitatorSystem.Goal agitatorGoal = AgitatorSystem.Goal.STOP;

        @Builder.Default
        private SliderSystem.Goal sliderGoal = SliderSystem.Goal.DEPLOYED;

        @Builder.Default
        private IntakeSystem.Goal intakeGoal = IntakeSystem.Goal.STOP;

        @Builder.Default
        @Getter
        private int priority = 1;

        private static LauncherSystem launcherSystem = null;
        private static AgitatorSystem agitatorSystem = null;
        private static SliderSystem sliderSystem = null;
        private static IntakeSystem intakeSystem = null;

        /* REQUIRED */
        public static void setStaticSubsystems(LauncherSystem ls, AgitatorSystem as, SliderSystem ss, IntakeSystem is) {
            launcherSystem = ls;
            agitatorSystem = as;
            sliderSystem = ss;
            intakeSystem = is;
        }

        public void set() {
            // set goals
            launcherSystem.setGoal(launcherGoal);
            agitatorSystem.setGoal(agitatorGoal);
            sliderSystem.setGoal(sliderGoal);
            intakeSystem.setGoal(intakeGoal);

            // run periodic()
            launcherSystem.periodic();
            agitatorSystem.periodic();
            sliderSystem.periodic();
            intakeSystem.periodic();
        }
    }
}
