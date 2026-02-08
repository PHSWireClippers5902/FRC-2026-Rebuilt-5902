package org.frc5902.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;

public class TwistUtil {
    public static final double kMinDt = 1e-6;

    public static Twist3d calculateTwist3d(Pose3d oldPose, Pose3d newPose, double dt) {
        // temp
        Rotation3d deltaRotation = newPose.getRotation().minus(oldPose.getRotation());
        return (dt <= kMinDt)
                ? new Twist3d()
                : new Twist3d(
                        (newPose.getX() - oldPose.getX()) / dt,
                        (newPose.getY() - oldPose.getY()) / dt,
                        (newPose.getZ() - oldPose.getZ()) / dt,
                        (deltaRotation.getX()) / dt,
                        (deltaRotation.getY()) / dt,
                        (deltaRotation.getZ()) / dt);
    }

    public static Twist3d calculateTwist3d(Pose3d oldPose, Pose3d newPose, double oldTimestamp, double newTimestamp) {
        double dt = newTimestamp - oldTimestamp;
        if (dt <= kMinDt) return new Twist3d();
        return calculateTwist3d(oldPose, newPose, dt);
    }
}
