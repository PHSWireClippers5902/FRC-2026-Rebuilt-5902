package org.frc5902.robot.subsystems.questnav;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Queue;

public class QuestIOReal implements QuestIO {
    public final QuestNav questNav;
    private final Queue<Double> timestampQueue;
    private final Queue<Pose3d> poseQueue;

    private final Debouncer questConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer questTrackingDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private volatile Pose3d latestPose = new Pose3d();

    public QuestIOReal() {
        questNav = new QuestNav();

        timestampQueue = QuestThread.getInstance().makeTimestampQueue();
        poseQueue = QuestThread.getInstance().registerSignal(() -> latestPose);
    }

    /** SUBSYSTEM NEEDS THIS... */
    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }

    public void updateInputs(QuestIOInputs inputs) {
        PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();
        for (PoseFrame frame : newFrames) {
            latestPose = frame.questPose3d();
        }

        inputs.connected = questConnectedDebounce.calculate(questNav.isConnected());
        inputs.isTracking = questTrackingDebounce.calculate(questNav.isTracking());
        inputs.batteryPercent = questNav.getBatteryPercent().orElse(-1);
        inputs.latency = questNav.getLatency();
        // read from queues (multi-threading...)
        inputs.questTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.readPoses = poseQueue.stream().toArray(Pose3d[]::new);

        // clear mah stuff
        timestampQueue.clear();
        poseQueue.clear();
    }

    public void setPose(Pose3d pose) {
        questNav.setPose(pose);
    }
}
