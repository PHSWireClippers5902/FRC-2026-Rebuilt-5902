package org.frc5902.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import lombok.Builder;

public class VisionConstants {
    public static CameraConfig[] cameras = new CameraConfig[]{};

    @Builder
    public record CameraConfig(
        Supplier<Pose3d> pose,
        String id,
        int width,
        int height,
        int autoExposure,
        int exposure,
        double gain,
        double denoise,
        double stdDevFactor) {}
    
}
