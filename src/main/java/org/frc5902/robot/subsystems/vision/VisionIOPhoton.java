package org.frc5902.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionIOPhoton implements VisionIO {
    // protected final Transform3d[] robotToCameras;
    protected final PhotonCamera camera;

    public VisionIOPhoton(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {}
}
