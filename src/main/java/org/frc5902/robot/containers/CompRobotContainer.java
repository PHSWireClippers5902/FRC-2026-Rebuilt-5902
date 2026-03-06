package org.frc5902.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.FieldConstants.AprilTagLayoutType;
import org.frc5902.robot.Robot;
import org.frc5902.robot.commands.auto.AutoBuilder;
import org.frc5902.robot.commands.auto.AutoPlease;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorIO;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorIOTalon;
import org.frc5902.robot.subsystems.compbot.agitator.AgitatorSystem;
import org.frc5902.robot.subsystems.compbot.intake.IntakeIO;
import org.frc5902.robot.subsystems.compbot.intake.IntakeIOSpark;
import org.frc5902.robot.subsystems.compbot.intake.IntakeSystem;
import org.frc5902.robot.subsystems.compbot.launcher.LauncherSystem;
import org.frc5902.robot.subsystems.compbot.launcher.flywheel.FlywheelIO;
import org.frc5902.robot.subsystems.compbot.launcher.flywheel.FlywheelIOSpark;
import org.frc5902.robot.subsystems.compbot.launcher.inserter.InserterIO;
import org.frc5902.robot.subsystems.compbot.launcher.inserter.InserterIOSpark;
import org.frc5902.robot.subsystems.compbot.slider.SliderIO;
import org.frc5902.robot.subsystems.compbot.slider.SliderIOSpark;
import org.frc5902.robot.subsystems.compbot.slider.SliderSystem;
import org.frc5902.robot.subsystems.compbot.superstructure.Superstructure;
import org.frc5902.robot.subsystems.compbot.superstructure.SuperstructureActions;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO_ADIS;
import org.frc5902.robot.subsystems.drive.modules.ModuleIO;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSim;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSparkAbsolute;
import org.frc5902.robot.subsystems.questnav.QuestIO;
import org.frc5902.robot.subsystems.questnav.QuestIOReal;
import org.frc5902.robot.subsystems.questnav.QuestSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CompRobotContainer extends RobotContainer {
    // init subsystems here
    private final Drive drive;
    private final Superstructure superstructure;
    private final AgitatorSystem agitator;
    private final LauncherSystem launcher;
    private final IntakeSystem intake;
    private final SliderSystem slider;

    @SuppressWarnings("unused")
    private final QuestSubsystem quest;
    // vision? implement later, pah-lease!
    // command xbox
    private final CommandXboxController m_XboxController = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;

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
                agitator = new AgitatorSystem(new AgitatorIOTalon());
                launcher = new LauncherSystem(new InserterIOSpark(), new FlywheelIOSpark(), this::getidealspin);
                intake = new IntakeSystem(new IntakeIOSpark());
                slider = new SliderSystem(new SliderIOSpark());
                quest = new QuestSubsystem(new QuestIOReal());
                superstructure = new Superstructure(agitator, intake, launcher, slider);
                break;
            case SIM:
                // sim bot
                drive = new Drive(
                        new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                agitator = new AgitatorSystem(new AgitatorIO() {});
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {}, this::getidealspin);
                intake = new IntakeSystem(new IntakeIO() {});
                slider = new SliderSystem(new SliderIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(agitator, intake, launcher, slider);
                break;
            default:
                // replay
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                agitator = new AgitatorSystem(new AgitatorIO() {});
                launcher = new LauncherSystem(new InserterIO() {}, new FlywheelIO() {}, this::getidealspin);
                intake = new IntakeSystem(new IntakeIO() {});
                slider = new SliderSystem(new SliderIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                superstructure = new Superstructure(agitator, intake, launcher, slider);
                break;
        }
        var autoBuilder = new AutoBuilder(drive, superstructure);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        // sysid routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("Auto pls work", AutoPlease.extendAndMoveAuto(() -> drive, () -> superstructure));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                () -> m_XboxController.getRightX(),
                () -> false));

        m_XboxController.povDown().onTrue(DriveCommands.resetGyroscope(drive));

        m_XboxController
                .rightTrigger(0.2)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.INTAKE))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.INTAKE));
        m_XboxController
                .rightBumper()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.OUTTAKE))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.OUTTAKE));
        m_XboxController
                .b()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.READY_LAUNCHER_STUPID))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.READY_LAUNCHER_STUPID));
        m_XboxController
                .leftTrigger(0.2)
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.LAUNCH_STUPID))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.LAUNCH_STUPID));
        m_XboxController
                .y()
                .onTrue(superstructure.addCommandToScheduler(SuperstructureActions.CLEAR_FLYWHEEL_JAM))
                .onFalse(superstructure.removeCommandFromScheduler(SuperstructureActions.CLEAR_FLYWHEEL_JAM));

        m_XboxController
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -m_XboxController.getLeftY(),
                        () -> -m_XboxController.getLeftX(),
                        () -> Rotation2d.fromDegrees(90)));

        m_XboxController.x().onTrue(DriveCommands.defenceGoal(drive));
    }

    public AprilTagLayoutType getSelectedAprilTagLayout() {
        return FieldConstants.defaultAprilTagType;
    }

    public double getidealspin() {
        return m_XboxController.getLeftTriggerAxis();
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
        return new Pose2d();
    }
}
