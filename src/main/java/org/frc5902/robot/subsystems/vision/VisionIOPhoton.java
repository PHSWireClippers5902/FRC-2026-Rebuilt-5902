package org.frc5902.robot.subsystems.vision;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.RobotState;
import org.frc5902.robot.RobotState.VisionObservation;
import org.photonvision.PhotonCamera;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class VisionIOPhoton implements VisionIO {

    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Short> tagIds = new HashSet<>();
        List<VisionObservation> poseObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                RobotState.getInstance().addVisionObservation(new VisionObservation(null, result.getTimestampSeconds(), null));
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Observation
                poseObservations.add(
                        new VisionObservation(
                                result.getTimestampSeconds(), // Timestamp
                                robotPose, // 3D pose estimate
                                multitagResult.estimatedPose.ambiguity, // Ambiguity
                                multitagResult.fiducialIDsUsed.size(), // Tag count
                                totalTagDistance / result.targets.size(), // Average tag distance
                                VisionObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);

                // Find robot pose
                var tagPose = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget =
                            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add((short) target.fiducialId);

                    poseObservations.add(
                            new VisionObservation( //not in Robot State @TODO Fix!
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                                    VisionObservationType.PHOTONVISION)); // Observation type

                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new VisionObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs object
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}
