package org.frc5902.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.FieldConstants.AprilTagLayoutType;
import org.frc5902.robot.Robot;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.commands.intake.IntakeCommands;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO;
import org.frc5902.robot.subsystems.drive.gyro.GyroIO_ADXRS;
import org.frc5902.robot.subsystems.drive.modules.ModuleIO;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSim;
import org.frc5902.robot.subsystems.drive.modules.ModuleIOSparkAbsolute;
import org.frc5902.robot.subsystems.kitbot.intake.Intake;
import org.frc5902.robot.subsystems.kitbot.intake.IntakeIO;
import org.frc5902.robot.subsystems.kitbot.intake.IntakeIOSim;
import org.frc5902.robot.subsystems.kitbot.intake.IntakeIOTalonSRX;
import org.frc5902.robot.subsystems.questnav.QuestIO;
import org.frc5902.robot.subsystems.questnav.QuestIOReal;
import org.frc5902.robot.subsystems.questnav.QuestSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class KitbotRobotContainer extends RobotContainer {
    // init subsystems here
    private final Drive drive;
    private final Intake intake;
    private final QuestSubsystem quest;
    // command xbox
    private final CommandXboxController m_XboxController = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    private final Alert primaryDisconnected = new Alert("Primary controller disconnected.", AlertType.kWarning);

    public KitbotRobotContainer() {
        switch (RobotConstants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIO_ADXRS(),
                        new ModuleIOSparkAbsolute(0),
                        new ModuleIOSparkAbsolute(1),
                        new ModuleIOSparkAbsolute(2),
                        new ModuleIOSparkAbsolute(3));
                intake = new Intake(new IntakeIOTalonSRX());
                quest = new QuestSubsystem(new QuestIOReal());
                break;
            case SIM:
                // sim bot
                drive = new Drive(
                        new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                intake = new Intake(new IntakeIOSim());
                quest = new QuestSubsystem(new QuestIO() {});
                break;
            default:
                // replay
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                intake = new Intake(new IntakeIO() {});
                quest = new QuestSubsystem(new QuestIO() {});
                break;
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // sysid routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                () -> m_XboxController.getRightX()));
        intake.setDefaultCommand(IntakeCommands.stopCommand(intake));

        m_XboxController
                .rightTrigger(0.1)
                .onTrue(IntakeCommands.intakeCommand(
                        intake, m_XboxController::getRightTriggerAxis, m_XboxController::getRightTriggerAxis));

        m_XboxController
                .leftTrigger(0.1)
                .onTrue(IntakeCommands.flywheelCommand(
                        intake, m_XboxController::getLeftTriggerAxis, m_XboxController::getLeftTriggerAxis));

        m_XboxController.a().whileTrue(IntakeCommands.spit(intake));

        m_XboxController.b().whileTrue(DriveCommands.resetGyroscope(drive));

        m_XboxController.x().whileTrue(DriveCommands.defenceGoal(drive));

        m_XboxController.y().whileTrue(DriveCommands.resetSwerveAbsolutePositions(drive));
    }

    public Pose2d getInitialPose() {
        return null;
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public AprilTagLayoutType getSelectedAprilTagLayout() {
        return FieldConstants.defaultAprilTagType;
    }

    @Override
    public void updateDashboardOutputs() {
        super.updateDashboardOutputs();

        primaryDisconnected.set(
                !DriverStation.isJoystickConnected(m_XboxController.getHID().getPort()));
    }
}
