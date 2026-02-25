package org.frc5902.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.state.RobotState.VisionObservation;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhoton implements VisionIO {
    // protected final Transform3d[] robotToCameras;
    protected final PhotonCamera camera;
    
    public VisionIOPhoton(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Short> tagIds = new HashSet<>();
        List<VisionObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                RobotState.getInstance().addVisionObservation(
                    new VisionObservation(null, result.getTimestampSeconds(), null)
                );
            }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // calc
            }
        }
    }
}
