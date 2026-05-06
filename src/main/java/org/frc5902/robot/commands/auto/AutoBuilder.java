package org.frc5902.robot.commands.auto;
/**
 * 
 * 
 * @author Daniel Sabalakov
 */



import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5902.robot.RobotState;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.questnav.QuestSubsystem;
import org.frc5902.robot.subsystems.superstructure.Superstructure;

public class AutoBuilder {

    private final Drive drive;
    private final Superstructure superstructure;
    private final QuestSubsystem quest;

    public AutoBuilder(Drive drive, Superstructure superstructure, QuestSubsystem quest) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.quest = quest;

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        com.pathplanner.lib.auto.AutoBuilder.configure(
                () -> RobotState.getInstance().getEstimatedPose(),
                (pose) -> {
                    quest.resetPose(pose);
                    // drive.resetGyroscope(pose.getRotation());
                    RobotState.getInstance().resetPose(pose);
                },
                drive::getRobotRelativeSpeeds,
                (speeds, feedfowards) -> drive.runVelocity(speeds),
                new PPHolonomicDriveController(new PIDConstants(6.0, 0.0, 0.0), new PIDConstants(6.0, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drive);
    }
}
