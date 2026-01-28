package org.frc5902.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5902.robot.Constants.RobotConstants;
import org.frc5902.robot.Robot;
import org.frc5902.robot.commands.drive.DriveCommands;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.drive.GyroIO;
import org.frc5902.robot.subsystems.drive.GyroIO_ADXRS;
import org.frc5902.robot.subsystems.drive.ModuleIO;
import org.frc5902.robot.subsystems.drive.ModuleIOSim;
import org.frc5902.robot.subsystems.drive.ModuleIOSparkRelative;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class KitbotRobotContainer extends RobotContainer {
    // init subsystems here
    private final Drive drive;

    // command xbox
    private final CommandXboxController m_XboxController = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public KitbotRobotContainer() {
        switch (RobotConstants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIO_ADXRS(),
                        new ModuleIOSparkRelative(0),
                        new ModuleIOSparkRelative(1),
                        new ModuleIOSparkRelative(2),
                        new ModuleIOSparkRelative(3));
                break;
            case SIM:
                // sim bot
                drive = new Drive(
                        new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                break;
            default:
                // replay
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                break;
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        configureBindings();
    }

    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -m_XboxController.getLeftY(),
                () -> -m_XboxController.getLeftX(),
                () -> -m_XboxController.getRightX()));
    }

    public Pose2d getInitialPose() {
        return null;
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
