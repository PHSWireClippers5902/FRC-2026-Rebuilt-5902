package org.frc5902.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO {
    @AutoLog
    public class QuestIOInputs {
        public boolean connected = false;
        public boolean isTracking = false;
        public int batteryPercent = -1;
        public double latency = 0.0;

        public double[] questTimestamps = new double[] {};
        public Pose3d[] readPoses = new Pose3d[] {};
    }

    public default void updateInputs(QuestIOInputs inputs) {}

    public default void setPose(Pose3d pose) {}

    public default void periodic() {}
}
