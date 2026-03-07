package org.frc5902.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CompbotConstants {
    public Transform3d robotCenter = new Transform3d(new Translation3d(), new Rotation3d());
    public Transform3d backLeftModule = new Transform3d(
            new Translation3d(-Units.inchesToMeters(27.125) / 2, Units.inchesToMeters(27.125) / 2, 0),
            new Rotation3d());
    // @ refer to robot center
    // millemeters
    public Transform3d questOffset = new Transform3d(
                    new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(-9), Units.inchesToMeters(24)),
                    new Rotation3d(0, 0, Math.PI))
            .plus(backLeftModule.inverse());
}
