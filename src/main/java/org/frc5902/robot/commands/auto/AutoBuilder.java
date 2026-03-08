package org.frc5902.robot.commands.auto;

import lombok.RequiredArgsConstructor;
import org.frc5902.robot.subsystems.compbot.superstructure.Superstructure;
import org.frc5902.robot.subsystems.drive.Drive;
import org.frc5902.robot.subsystems.questnav.QuestSubsystem;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;


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
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        
        com.pathplanner.lib.auto.AutoBuilder.configure(
            () -> new Pose2d(quest.getLatestPose().getTranslation().toTranslation2d(), drive.getGyroRotation()),
            (pose) -> {
                quest.resetPose(pose);
                drive.resetGyroscope();
            },
            null, 
            (speeds, feedfowards) -> System.out.println("TEST"), 
            new PPHolonomicDriveController(new PIDConstants(5.0,0.0,0.0), new PIDConstants(5.0, 0.0, 0.0)), 
            config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, drive);
    }

}
