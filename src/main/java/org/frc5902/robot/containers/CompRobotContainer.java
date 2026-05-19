package org.frc5902.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.FieldConstants.AprilTagLayoutType;
import org.frc5902.robot.Robot;
import org.frc5902.robot.RobotState;
import org.frc5902.robot.commands.MathCommands.SinusoidalControlCommand;
import org.frc5902.robot.commands.MiscCommands;
import org.frc5902.robot.commands.auto.AutoBuilder;
import org.frc5902.robot.commands.auto.EasyAutonomousCommandFactory;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.commands.drive.PointCommands;
import org.frc5902.robot.subsystems.SLAMtake.SLAMTake;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIO;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIOSpark;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIO;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIOSpark;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO_ADIS;
import org.frc5902.robot.subsystems.drive.modules.ModuleIO;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSim;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSparkAbsolute;
import org.frc5902.robot.subsystems.indexer.IndexerIO;
import org.frc5902.robot.subsystems.indexer.IndexerIOSpark;
import org.frc5902.robot.subsystems.indexer.IndexerSystem;
import org.frc5902.robot.subsystems.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.launcher.flywheel.FlywheelIO;
import org.frc5902.robot.subsystems.launcher.flywheel.FlywheelIOSparks;
import org.frc5902.robot.subsystems.launcher.inserter.InserterIO;
import org.frc5902.robot.subsystems.launcher.inserter.InserterIOSpark;
import org.frc5902.robot.subsystems.questnav.QuestIO;
import org.frc5902.robot.subsystems.questnav.QuestIOReal;
import org.frc5902.robot.subsystems.questnav.QuestSubsystem;
import org.frc5902.robot.subsystems.rumble.Rumble;
import org.frc5902.robot.subsystems.superstructure.Superstructure;
import org.frc5902.robot.subsystems.superstructure.SuperstructureActions;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

public class CompRobotContainer extends RobotContainer {
    // init subsystems here
    private final Drive drive;
    private final Superstructure superstructure;
    private final LauncherSystem launcher;
    private final IndexerSystem indexer;
    private final SLAMTake slamTake;

    private final QuestSubsystem quest;

    @Getter
    @Setter
    private boolean OVERRIDE_SMART_LAUNCH = false;
    // vision? implement later, pah-lease!
    // command xbox
    private final CommandXboxController m_XboxController = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Pose2d> initialPositionChooser;

    private final Alert primaryDisconnected = new Alert("Primary controller disconnected.", AlertType.kWarning);

    public CompRobotContainer() {
        switch (RobotConstants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIO_ADIS(),
                        new ModuleIOSparkAbsolute(0),
                        new ModuleIOSparkAbsolute(1),
                        new ModuleIOSparkAbsolute(2),
                        new ModuleIOSparkAbsolute(3));
                launcher = new LauncherSystem(new InserterIOSpark(), new FlywheelIOSparks());
                slamTake = new SLAMTake(new SlamIOSpark(), new IntakeIOSpark());
                indexer = new IndexerSystem(new IndexerIOSpark());
                quest = new QuestSubsystem(new QuestIOReal());
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
            case SIM:
                // sim bot
                drive = new Drive(
                        new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {});
                slamTake = new SLAMTake(new SlamIO() {}, new IntakeIO() {});
                indexer = new IndexerSystem(new IndexerIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
            default:
                // replay
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {});
                slamTake = new SLAMTake(new SlamIO() {}, new IntakeIO() {});
                indexer = new IndexerSystem(new IndexerIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
        }
        // var autoBuilder = new AutoBuilder(drive, superstructure);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        initialPositionChooser = new LoggedDashboardChooser<>("Initial Positions");

        initialPositionChooser.addOption("BLUE_LEFT_TRENCH", new Pose2d(4.147, 7.642, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_LEFT_TRENCH", new Pose2d(12.434, 0.407, Rotation2d.kZero));
        initialPositionChooser.addOption("BLUE_RIGHT_TRENCH", new Pose2d(4.147, 0.407, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_RIGHT_TRENCH", new Pose2d(12.434, 7.642, Rotation2d.kZero));

        initialPositionChooser.addOption("BLUE_LEFT_BUMP", new Pose2d(3.622, 5.066, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_LEFT_BUMP", new Pose2d(12.908, 3.015, Rotation2d.kZero));
        initialPositionChooser.addOption("BLUE_RIGHT_BUMP", new Pose2d(3.622, 3.015, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_RIGHT_BUMP", new Pose2d(12.908, 5.066, Rotation2d.k180deg));

        initialPositionChooser.addOption("BLUE_HUB_POS", new Pose2d(3.560, 4.1, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_HUB_POS", new Pose2d(12.939, 4.1, Rotation2d.kZero));

        // sysid routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("FORWARD PLEASE", EasyAutonomousCommandFactory.forwardAuto(() -> drive));
        autoChooser.addOption("DO NOTHING", EasyAutonomousCommandFactory.doNothingAuto(() -> drive));

        // pathplanner
        @SuppressWarnings("unused")
        AutoBuilder ab = new AutoBuilder(drive, superstructure, quest);

        NamedCommands.registerCommand("RESET_POSE", MiscCommands.PoseResetCommand(this));
        NamedCommands.registerCommand(
                "OSCILLATE",
                PointCommands.pointAtTranslation(drive, FieldConstants.BLUE_HUB_LOCATION)
                        .repeatedly());
        NamedCommands.registerCommand(
                "INTAKE",
                Superstructure.AutonomousSuperstructureGoalCommand(SuperstructureActions.INTAKE, 1000, superstructure));
        NamedCommands.registerCommand(
                "READY_LAUNCHER",
                Superstructure.AutonomousSuperstructureGoalCommand(
                        SuperstructureActions.READY_LAUNCHER_STUPID, 1000, superstructure));
        NamedCommands.registerCommand(
                "LAUNCH",
                Superstructure.AutonomousSuperstructureGoalCommand(
                        SuperstructureActions.LAUNCH_STUPID, 3.0, superstructure));

        NamedCommands.registerCommand(
                "LAUNCH_SEQUENCE", MiscCommands.LaunchSequenceBetterSTUPID(drive, superstructure));
        //        public static double raisedAngleLOW = 0.09;
        NamedCommands.registerCommand(
                "SINUSOIDAL_SLAM",
                new SinusoidalControlCommand(
                        null,
                        (value) -> superstructure.getSlam().runSLAMAngle(value),
                        "SINUSOIDAL_SLAM",
                        -0.07,
                        2.5,
                        0,
                        0.07,
                        -1,
                        0.0,
                        true));

        // GREEDY SWEEP
        autoChooser.addOption("BLUE_LEFT_GREEDY_SWEEP", new PathPlannerAuto("SWEEP_GREEDY_L"));
        autoChooser.addOption("RED_RIGHT_GREEDY_SWEEP", new PathPlannerAuto("SWEEP_GREEDY_L", true));
        autoChooser.addOption("RED_LEFT_GREEDY_SWEEP", new PathPlannerAuto("SWEEP_GREEDY_L"));
        autoChooser.addOption("BLUE_RIGHT_GREEDY_SWEEP", new PathPlannerAuto("SWEEP_GREEDY_L", true));
        // GREEDY SHOOT
        autoChooser.addOption("BLUE_LEFT_GREEDY_SHOOT", new PathPlannerAuto("SHOOT_GREEDY_L"));
        autoChooser.addOption("RED_RIGHT_GREEDY_SHOOT", new PathPlannerAuto("SHOOT_GREEDY_L", true));
        autoChooser.addOption("RED_LEFT_GREEDY_SHOOT", new PathPlannerAuto("SHOOT_GREEDY_L"));
        autoChooser.addOption("BLUE_RIGHT_GREEDY_SHOOT", new PathPlannerAuto("SHOOT_GREEDY_L", true));
        // TRENCH SHOOT
        autoChooser.addOption("BLUE_LEFT_TRENCH_SHOOT", new PathPlannerAuto("SHOOT_TRENCH_L"));
        autoChooser.addOption("RED_RIGHT_TRENCH_SHOOT", new PathPlannerAuto("SHOOT_TRENCH_L", true));
        autoChooser.addOption("RED_LEFT_TRENCH_SHOOT", new PathPlannerAuto("SHOOT_TRENCH_L"));
        autoChooser.addOption("BLUE_RIGHT_TRENCH_SHOOT", new PathPlannerAuto("SHOOT_TRENCH_L", true));
        // TRENCH SWEEP
        autoChooser.addOption("BLUE_LEFT_TRENCH_SWEEP", new PathPlannerAuto("SWEEP_TRENCH_L"));
        autoChooser.addOption("RED_RIGHT_TRENCH_SWEEP", new PathPlannerAuto("SWEEP_TRENCH_L", true));
        autoChooser.addOption("RED_LEFT_TRENCH_SWEEP", new PathPlannerAuto("SWEEP_TRENCH_L"));
        autoChooser.addOption("BLUE_RIGHT_TRENCH_SWEEP", new PathPlannerAuto("SWEEP_TRENCH_L", true));

        // CENTER SHOOT
        autoChooser.addOption("BLUE_LEFT_CENTER_SHOOT", new PathPlannerAuto("CENTER_SHOOT_GREEDY_L"));
        autoChooser.addOption("RED_RIGHT_CENTER_SHOOT", new PathPlannerAuto("CENTER_SHOOT_GREEDY_L", true));
        autoChooser.addOption("RED_LEFT_CENTER_SHOOT", new PathPlannerAuto("CENTER_SHOOT_GREEDY_L"));
        autoChooser.addOption("BLUE_RIGHT_CENTER_SHOOT", new PathPlannerAuto("CENTER_SHOOT_GREEDY_L", true));
        // CENTER SWEEP
        autoChooser.addOption("BLUE_LEFT_CENTER_SWEEP", new PathPlannerAuto("CENTER_SWEEP_GREEDY_L"));
        autoChooser.addOption("RED_RIGHT_CENTER_SWEEP", new PathPlannerAuto("CENTER_SWEEP_GREEDY_L", true));
        autoChooser.addOption("RED_LEFT_CENTER_SWEEP", new PathPlannerAuto("CENTER_SWEEP_GREEDY_L"));
        autoChooser.addOption("BLUE_RIGHT_CENTER_SWEEP", new PathPlannerAuto("CENTER_SWEEP_GREEDY_L", true));

        autoChooser.addOption("BLUE_LEFT_PLAYOFF_TRENCH", new PathPlannerAuto("SWEEP_TRENCH_L_ONE"));
        autoChooser.addOption("RED_RIGHT_PLAYOFF_TRENCH", new PathPlannerAuto("SWEEP_TRENCH_L_ONE", true));
        autoChooser.addOption("RED_LEFT_PLAYOFF_TRENCH", new PathPlannerAuto("SWEEP_TRENCH_L_ONE"));
        autoChooser.addOption("BLUE_RIGHT_PLAYOFF_TRENCH", new PathPlannerAuto("SWEEP_TRENCH_L_ONE", true));

        // autoChooser.addOption("BLUE_LEFT_CENTER");
        // autoChooser.addOption("BLUE_LEFT_CENTER");
        // autoChooser.addOption("BLUE_LEFT_CENTER");
        // autoChooser.addOption("BLUE_LEFT_CENTER");

        // autoChooser.addOption("Auto pls work", AutoPlease.extendAndMoveAuto(() -> drive, () -> superstructure));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        initialPositionChooser.addDefaultOption("default (0,0)", new Pose2d());

        Rumble.getInstance(Optional.of(m_XboxController));

        configureBindings();
    }

    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        // set default commands here.... here I say.... HERE
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -m_XboxController.getLeftY(),
                () -> -m_XboxController.getLeftX(),
                () -> -m_XboxController.getRightX(),
                () -> false,
                0.23,
                0.5));

        // m_XboxController.rightStick().onTrue(DriveCommands.resetGyroscope(drive));

        m_XboxController
                .rightTrigger(0.2)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.INTAKE))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.INTAKE));

        // m_XboxController
        //         .b()
        //         .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.READY_LAUNCHER_STUPID))
        //         .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.READY_LAUNCHER_STUPID));
        m_XboxController
                .leftTrigger(0.2)
                .and(() -> !this.OVERRIDE_SMART_LAUNCH)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.READY_SMART))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.READY_SMART));

        m_XboxController
                .leftBumper()
                .and(() -> !this.OVERRIDE_SMART_LAUNCH)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.LAUNCH_SMART))
                .whileTrue(Commands.parallel(
                        NamedCommands.getCommand("SINUSOIDAL_SLAM"),
                        PointCommands.pointAtTranslationWhileMoving(
                                drive,
                                () -> -m_XboxController.getLeftY(),
                                () -> -m_XboxController.getLeftX(),
                                0.23,
                                FieldConstants.BLUE_HUB_LOCATION)))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.LAUNCH_SMART));

        m_XboxController
                .leftTrigger(0.2)
                .and(() -> this.OVERRIDE_SMART_LAUNCH)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.READY_LAUNCHER_STUPID))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.READY_LAUNCHER_STUPID));

        m_XboxController
                .leftBumper()
                .and(() -> this.OVERRIDE_SMART_LAUNCH)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.LAUNCH_STUPID))
                .whileTrue(NamedCommands.getCommand("SINUSOIDAL_SLAM"))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.LAUNCH_STUPID));
        // m_XboxController
        //         .b()
        //         .toggleOnTrue(Commands.runOnce(() -> this.OVERRIDE_SMART_LAUNCH = !this.OVERRIDE_SMART_LAUNCH));
        m_XboxController.b().whileTrue(PointCommands.pointAtTranslation(drive, FieldConstants.BLUE_HUB_LOCATION));

        // .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.RAISED_INTAKE))
        // .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.RAISED_INTAKE));
        m_XboxController
                .y()
                .toggleOnTrue(new ConditionalCommand(
                        superstructure.addCommandToScheduler(SuperstructureActions.RAISED_INTAKE_HIGH),
                        superstructure.removeCommandFromScheduler(SuperstructureActions.RAISED_INTAKE_HIGH),
                        () -> !superstructure.isScheduled(SuperstructureActions.RAISED_INTAKE_HIGH)));
        // .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.RAISED_INTAKE_HIGH));

        // m_XboxController
        //         .a()
        //         .whileTrue(DriveCommands.pointAtPoseWhileMoving(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 new Pose2d(AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_LOCATION), new Rotation2d())));

        // m_XboxController.x().whileTrue(DriveCommands.aimAtAngleOSCLILATE(drive, () -> drive.getGyroRotation()));
        m_XboxController
                .x()
                .whileTrue(new SinusoidalControlCommand(
                        drive,
                        (value) -> drive.runVelocity(new ChassisSpeeds(0, 0, value)),
                        "RotationSine",
                        0.3,
                        0.5,
                        0,
                        0,
                        -1,
                        0.0,
                        false));

        m_XboxController
                .a()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.INVERT_FEED))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.INVERT_FEED));
        // m_XboxController
        //         .b()
        //         .whileTrue(DriveCommands.aimAtAngle(drive, () -> LauncherCalculatorExperimental.calcPointedAngle()))
        //         .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.SMART_LAUNCH))
        //         .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.SMART_LAUNCH));
        // m_XboxController
        //         .y()
        //         .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.MOVE_INTAKE_DOWN))
        //         .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.MOVE_INTAKE_DOWN));

        // m_XboxController
        //         .a()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> Rotation2d.fromDegrees(90)));

        // m_XboxController
        //         .povLeft()
        //         .whileTrue(DriveCommands.joystickDriveAround(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> -m_XboxController.getRightX(),
        //                 () -> false,
        //                 new Translation2d(
        //                         DriveConstants.ModuleConfigurations.driveBaseRadius,
        //                         DriveConstants.ModuleConfigurations.driveBaseRadius)));

        // m_XboxController
        //         .povRight()
        //         .whileTrue(DriveCommands.joystickDriveAround(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> -m_XboxController.getRightX(),
        //                 () -> false,
        //                 new Translation2d(
        //                         -DriveConstants.ModuleConfigurations.driveBaseRadius,
        //                         -DriveConstants.ModuleConfigurations.driveBaseRadius)));

        // m_XboxController
        //         .povUp()
        //         .whileTrue(DriveCommands.joystickDriveAround(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> -m_XboxController.getRightX(),
        //                 () -> false,
        //                 new Translation2d(
        //                         DriveConstants.ModuleConfigurations.driveBaseRadius,
        //                         -DriveConstants.ModuleConfigurations.driveBaseRadius)));

        // m_XboxController
        //         .povDown()
        //         .whileTrue(DriveCommands.joystickDriveAround(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> -m_XboxController.getRightX(),
        //                 () -> false,
        //                 new Translation2d(
        //                         -DriveConstants.ModuleConfigurations.driveBaseRadius,
        //                         DriveConstants.ModuleConfigurations.driveBaseRadius)));

        m_XboxController.button(8).onTrue(MiscCommands.PoseResetCommand(this));
        m_XboxController.button(7).toggleOnTrue(Superstructure.getInstance().SWAP_KILL_SYSTEMS());

        m_XboxController
                .rightBumper()
                .whileTrue(DriveCommands.joystickDrive(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> -m_XboxController.getRightX(),
                        () -> false,
                        1.0,
                        1.0));

        // m_XboxController.x().whileTrue(DriveCommands.defenseGoal(drive));
    }

    public AprilTagLayoutType getSelectedAprilTagLayout() {
        return FieldConstants.defaultAprilTagType;
    }

    @Override
    public void updateDashboardOutputs() {
        super.updateDashboardOutputs();
    }

    @Override
    public void updateAlerts() {
        primaryDisconnected.set(
                !DriverStation.isJoystickConnected(m_XboxController.getHID().getPort()));
    }

    @Override
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    @Override
    public Pose2d getInitialPose() {
        // fake it till u make it
        // depends on where you are....

        return initialPositionChooser.get();
    }

    @Override
    public void resetInitialPose() {
        RobotState.getInstance().resetPose(getInitialPose());
        quest.resetPose(getInitialPose());
        // drive.resetGyroscope(getInitialPose().getRotation());
    }
}
