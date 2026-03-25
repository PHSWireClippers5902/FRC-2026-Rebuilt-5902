package org.frc5902.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.FieldConstants.AprilTagLayoutType;
import org.frc5902.robot.Robot;
import org.frc5902.robot.RobotState;
import org.frc5902.robot.commands.auto.EasyAutonomousCommandFactory;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.subsystems.SLAMtake.SLAMTake;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIO;
import org.frc5902.robot.subsystems.SLAMtake.intake.IntakeIOSpark;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIO;
import org.frc5902.robot.subsystems.SLAMtake.slam.SlamIOSpark;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.drive.DriveConstants;
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
import org.frc5902.robot.subsystems.launcher.flywheel.FlywheelIOSpark;
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

    @SuppressWarnings("unused")
    private final QuestSubsystem quest;
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
                launcher = new LauncherSystem(new InserterIOSpark(), new FlywheelIOSpark(0), new FlywheelIOSpark(1));
                slamTake = new SLAMTake(new SlamIOSpark(), new IntakeIOSpark());
                indexer = new IndexerSystem(new IndexerIOSpark());
                quest = new QuestSubsystem(new QuestIOReal());
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
            case SIM:
                // sim bot
                drive = new Drive(
                        new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {}, new FlywheelIO() {});
                slamTake = new SLAMTake(new SlamIO() {}, new IntakeIO() {});
                indexer = new IndexerSystem(new IndexerIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
            default:
                // replay
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {}, new FlywheelIO() {});
                slamTake = new SLAMTake(new SlamIO() {}, new IntakeIO() {});
                indexer = new IndexerSystem(new IndexerIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(slamTake, launcher, indexer);
                break;
        }
        // var autoBuilder = new AutoBuilder(drive, superstructure);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        initialPositionChooser = new LoggedDashboardChooser<>("Initial Positions");
        initialPositionChooser.addOption("BLUE_LEFT_BUMP", new Pose2d(3.56, 5.024, Rotation2d.kZero));
        initialPositionChooser.addOption("BLUE_RIGHT_BUMP", new Pose2d(3.56, 3.035, Rotation2d.kZero));
        initialPositionChooser.addOption("BLUE_CENTER", new Pose2d(3.56, 4.056, Rotation2d.k180deg));

        initialPositionChooser.addOption("RED_RIGHT_BUMP", new Pose2d(13, 5.024, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_LEFT_BUMP", new Pose2d(13, 3.035, Rotation2d.k180deg));
        initialPositionChooser.addOption("RED_CENTER", new Pose2d(13, 4.056, Rotation2d.kZero));

        // sysid routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("FORWARD PLEASE", EasyAutonomousCommandFactory.forwardAuto(() -> drive));
        autoChooser.addOption("DO NOTHING", EasyAutonomousCommandFactory.doNothingAuto(() -> drive));
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
                0.1,
                0.5));

        m_XboxController.rightStick().onTrue(DriveCommands.resetGyroscope(drive));

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
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.READY_LAUNCHER_STUPID))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.READY_LAUNCHER_STUPID));

        m_XboxController
                .leftBumper()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.LAUNCH_STUPID))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.LAUNCH_STUPID));

        m_XboxController
                .b()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.RAISED_INTAKE))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.RAISED_INTAKE));

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

        m_XboxController
                .povLeft()
                .whileTrue(DriveCommands.joystickDriveAround(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> -m_XboxController.getRightX(),
                        () -> false,
                        new Translation2d(
                                DriveConstants.ModuleConfigurations.driveBaseRadius,
                                DriveConstants.ModuleConfigurations.driveBaseRadius)));

        m_XboxController
                .povRight()
                .whileTrue(DriveCommands.joystickDriveAround(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> -m_XboxController.getRightX(),
                        () -> false,
                        new Translation2d(
                                -DriveConstants.ModuleConfigurations.driveBaseRadius,
                                -DriveConstants.ModuleConfigurations.driveBaseRadius)));

        m_XboxController
                .povUp()
                .whileTrue(DriveCommands.joystickDriveAround(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> -m_XboxController.getRightX(),
                        () -> false,
                        new Translation2d(
                                DriveConstants.ModuleConfigurations.driveBaseRadius,
                                -DriveConstants.ModuleConfigurations.driveBaseRadius)));

        m_XboxController
                .povDown()
                .whileTrue(DriveCommands.joystickDriveAround(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> -m_XboxController.getRightX(),
                        () -> false,
                        new Translation2d(
                                -DriveConstants.ModuleConfigurations.driveBaseRadius,
                                DriveConstants.ModuleConfigurations.driveBaseRadius)));

        // m_XboxController
        //         .leftBumper()
        //         .whileTrue(DriveCommands.joystickDrive(
        //                 drive,
        //                 () -> -m_XboxController.getLeftY(),
        //                 () -> -m_XboxController.getLeftX(),
        //                 () -> m_XboxController.getRightX(),
        //                 () -> false,
        //                 1.0,
        //                 1.0));

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
        drive.resetGyroscope(getInitialPose().getRotation());
    }
}
