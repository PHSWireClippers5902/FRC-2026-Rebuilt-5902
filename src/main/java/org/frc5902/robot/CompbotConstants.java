/**
 * @team: The Wire Clippers 5902
 * @name:    CompbotConstants
 * @purpose: A single place to store competition robot-wide constants. Updated for 2026.
 * @author:    Daniel Sabalakov
 */
package org.frc5902.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CompbotConstants {
    public static Transform3d robotCenter = new Transform3d(new Translation3d(), new Rotation3d());
    public static Transform3d backLeftModule = new Transform3d(
            new Translation3d(-Units.inchesToMeters(27.125) / 2, Units.inchesToMeters(27.125) / 2, 0),
            new Rotation3d());
    // @ refer to robot center
    // millemeters
    public static Transform3d questOffset = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(-6.9375), Units.inchesToMeters(12.0875), Units.inchesToMeters(18.25)),
            new Rotation3d(0, 0, Units.degreesToRadians(110)));
}
