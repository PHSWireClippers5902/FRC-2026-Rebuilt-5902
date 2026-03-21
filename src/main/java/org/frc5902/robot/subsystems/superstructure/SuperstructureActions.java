package org.frc5902.robot.subsystems.superstructure;

import lombok.Builder;
import lombok.Getter;
import lombok.ToString;
import org.frc5902.robot.subsystems.SLAMtake.SLAMTake;
import org.frc5902.robot.subsystems.indexer.IndexerSystem;
import org.frc5902.robot.subsystems.launcher.LauncherSystem;

public class SuperstructureActions {

    // STOW: Maintain defaults BUT have STOW
    public static SuperstructureAction STOW =
            SuperstructureAction.builder().priority(0).build();
    public static SuperstructureAction MOVE_INTAKE_UP = SuperstructureAction.builder()
            .priority(1)
            .slamGoal(SLAMTake.Goal.RAISE_STUPID)
            .build();
    public static SuperstructureAction MOVE_INTAKE_DOWN = SuperstructureAction.builder()
            .priority(1)
            .slamGoal(SLAMTake.Goal.LOWER_STUPID)
            .build();
    // DEPLOY_IDLE: Maintain all defaults
    public static SuperstructureAction DEPLOY_IDLE =
            SuperstructureAction.builder().priority(0).build();
    // INTAKE: Run Intake and Agitate Out
    public static SuperstructureAction INTAKE = SuperstructureAction.builder()
            .slamGoal(SLAMTake.Goal.LOWERED_INTAKE)
            .priority(1)
            .build();
    // INTAKE AND PASS: Run intake, flywheel, and agitator in
    public static SuperstructureAction INTAKE_AND_PASS = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.LAUNCH)
            .priority(2)
            .build();
    // OUTTAKE: Clear by kicking agitator AND running Outtake
    public static SuperstructureAction OUTTAKE =
            SuperstructureAction.builder().priority(1).build();
    // READY_LAUNCHER: Runs launcher at ready state.
    public static SuperstructureAction READY_LAUNCHER = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.READY)
            .priority(2)
            .build();
    // LAUNCH: Run agitate in, intake at low to keep fuel in, and launch system
    public static SuperstructureAction LAUNCH = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.LAUNCH)
            .priority(3)
            .build();
    // CLEAR_FLYWHEEL_JAM: Run agitator to kick fuel out, run intake in, and run launcher system to clear jam
    public static SuperstructureAction CLEAR_FLYWHEEL_JAM = SuperstructureAction.builder()
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
            .launcherGoal(LauncherSystem.Goal.LAUNCH_STUPID)
            .indexerGoal(IndexerSystem.Goal.MOVE_IN)
            .priority(3)
            .build();

    public static SuperstructureAction EMERGENCY = SuperstructureAction.builder()
            .launcherGoal(LauncherSystem.Goal.IDLE)
            .build();

    @Builder
    @ToString
    public static class SuperstructureAction {
        // Defaults for all actions
        @Builder.Default
        private LauncherSystem.Goal launcherGoal = LauncherSystem.Goal.IDLE;

        @Builder.Default
        private SLAMTake.Goal slamGoal = SLAMTake.Goal.STOP;

        @Builder.Default
        private IndexerSystem.Goal indexerGoal = IndexerSystem.Goal.STOP;

        @Builder.Default
        @Getter
        private int priority = 1;

        private static LauncherSystem launcherSystem = null;
        private static SLAMTake slamSystem = null;
        private static IndexerSystem indexerSystem = null;

        /* REQUIRED */
        public static void setStaticSubsystems(LauncherSystem ls, SLAMTake ss, IndexerSystem is) {
            launcherSystem = ls;
            slamSystem = ss;
            indexerSystem = is;
        }

        public void set() {
            // set goals
            launcherSystem.setGoal(launcherGoal);
            slamSystem.setGoal(slamGoal);
            indexerSystem.setGoal(indexerGoal);

            // run periodic()
            launcherSystem.periodic();
            slamSystem.periodic();
            indexerSystem.periodic();
        }
    }
}
