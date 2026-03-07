package org.frc5902.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AutoConstants {
    public static Pose3d RED_LEFT = new Pose3d(new Translation3d(), new Rotation3d());
    public static Pose3d RED_CENTER = new Pose3d(new Translation3d(), new Rotation3d());
    public static Pose3d RED_RIGHT = new Pose3d(new Translation3d(), new Rotation3d());

    public static Pose3d BLUE_LEFT = new Pose3d(new Translation3d(), new Rotation3d());
    public static Pose3d BLUE_CENTER = new Pose3d(new Translation3d(), new Rotation3d());
    public static Pose3d BLUE_RIGHT = new Pose3d(new Translation3d(), new Rotation3d());
}
