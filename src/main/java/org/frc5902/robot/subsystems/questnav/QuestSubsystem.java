package org.frc5902.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5902.robot.Constants.QuestConstants;
import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.state.RobotState.QuestObservation;
import org.frc5902.robot.util.GeoUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class QuestSubsystem extends SubsystemBase {
    public static final Lock questLock = new ReentrantLock();
    private final QuestIO questIO;
    private final QuestIOInputsAutoLogged questIOInputs = new QuestIOInputsAutoLogged();
    private final Alert questDisconnectedAlert = new Alert("The Quest has disconnected.", AlertType.kError);
    private final Alert questStoppedTrackingAlert =
            new Alert("The Quest has stopped tracking. Tracking may or may not resume.", AlertType.kError);

    public QuestSubsystem(QuestIO io) {
        this.questIO = io;
        QuestThread.getInstance().start();
    }

    @Override
    public void periodic() {
        questLock.lock();
        questIO.periodic();
        questIO.updateInputs(questIOInputs);
        Logger.processInputs("QuestNav", questIOInputs);
        questLock.unlock();
        if (questIOInputs.connected
                && questIOInputs.isTracking
                && questIOInputs.questTimestamps.length > 2
                && questIOInputs.readPoses.length > 1) {
            RobotState.getInstance()
                    .addQuestObservation(new QuestObservation(
                            getLatestPose(),
                            getLatestTwist3d(),
                            questIOInputs.isTracking,
                            questIOInputs.questTimestamps[questIOInputs.questTimestamps.length - 1]));
        }
        // alert
        questDisconnectedAlert.set(!questIOInputs.connected);
        questStoppedTrackingAlert.set(!questIOInputs.isTracking);
    }

    @AutoLogOutput(key = "QuestNav/Pose/QUEST")
    public Pose3d getLatestPose() {
        if (questIOInputs.readPoses.length < 1) return new Pose3d();
        return questIOInputs.readPoses[questIOInputs.readPoses.length - 1];
    }

    @AutoLogOutput(key = "QuestNav/Twist/QUEST")
    public Twist3d getLatestTwist3d() {
        // Ensure in-sync results
        int poseCount = questIOInputs.readPoses.length;
        int timeCount = questIOInputs.questTimestamps.length;
        if (poseCount < 2 || timeCount < 2) {
            return new Twist3d();
        }
        int i = Math.min(poseCount, timeCount);
        // avoid out of bounds errors people!
        Pose3d oldPose = questIOInputs.readPoses[i - 2];
        Pose3d newPose = questIOInputs.readPoses[i - 1];
        double oldTimestamp = questIOInputs.questTimestamps[i - 2];
        double newTimestamp = questIOInputs.questTimestamps[i - 1];
        return GeoUtil.calculateTwist3d(oldPose, newPose, oldTimestamp, newTimestamp);
    }

    @AutoLogOutput(key = "QuestNav/Pose/ROBOT")
    public Pose3d getLatestPoseRobotRelative() {
        if (questIOInputs.readPoses.length < 1) return new Pose3d();
        return (questIOInputs.readPoses[questIOInputs.readPoses.length - 1])
                .transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
    }

    @AutoLogOutput(key = "QuestNav/Twist/ROBOT")
    public Twist3d getLatestTwist3dRobotRelative() {
        // Ensure in-sync results
        int poseCount = questIOInputs.readPoses.length;
        int timeCount = questIOInputs.questTimestamps.length;
        if (poseCount < 2 || timeCount < 2) {
            return new Twist3d();
        }
        int i = Math.min(poseCount, timeCount);
        // avoid out of bounds errors people!
        Pose3d oldPose = questIOInputs.readPoses[i - 2].transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
        Pose3d newPose = questIOInputs.readPoses[i - 1].transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
        double oldTimestamp = questIOInputs.questTimestamps[i - 2];
        double newTimestamp = questIOInputs.questTimestamps[i - 1];
        return GeoUtil.calculateTwist3d(oldPose, newPose, oldTimestamp, newTimestamp);
    }
}
