package org.frc5902.robot.subsystems.vision;
import org.frc5902.robot.RobotState.VisionObservation;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public double[] timestamps = new double[0];
        public double[][] frames = new double[0][0];
        public int[] tagIds = new int[0];
        public VisionObservation[] poseObservations = new VisionObservation[0];
        public long fps = 0;
    }

    public static record PoseObservations(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type) {}

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

}
