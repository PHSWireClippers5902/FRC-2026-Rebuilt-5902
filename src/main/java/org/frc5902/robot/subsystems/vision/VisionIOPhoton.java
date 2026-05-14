package org.frc5902.robot.subsystems.vision;

import org.frc5902.robot.RobotState;
import org.frc5902.robot.RobotState.VisionObservation;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

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
            // if (result.hasTargets()) {
            //     RobotState.getInstance()
            //             .addVisionObservation(
            //                 new VisionObservation(result.getBestTarget().getBestCameraToTarget(), result.getTimestampSeconds(), null));
            // }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // calc

                Pose3d robotPose = new Pose3d();

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                tagIds.addAll(multitagResult.fiducialIDsUsed);

                RobotState.getInstance().addVisionObservation(
                    new VisionObservation(robotPose, result.getTimestampSeconds(), null)
                );

            }
            else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);
                // var tagPose = .getTagPose(target.fiducialId);
                
            }
        }
    }
}
