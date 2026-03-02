/**
 * Flywheel Constants are to be tuned in the robot.
 * @blame Daniel Sabalakov
 * @todo Confirm measurements, integrate into wpilib by making a sample robot project.
 */
package org.frc5902.robot.util.flywheellib.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.frc5902.robot.util.flywheellib.functions.BaseConstant;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;

public class FlywheelConstants {
    public static double FlywheelMountedAngleRadians = Math.PI / 6;
    // 7.5, 8, 17, from BACK RIGHT
    // length robot is 27
    public static Transform3d botToFlywheel = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(13.5 - 7.5), Units.inchesToMeters(13.5 + 8), Units.inchesToMeters(17)),
            new Rotation3d(0, 0, -Math.PI / 2));
    public static Transform3d flywheelToBot = botToFlywheel.inverse();

    public static Transform2d botToFlywheel2d = new Transform2d(
            new Translation2d(botToFlywheel.getX(), botToFlywheel.getY()), new Rotation2d(-Math.PI / 2));
    public static Transform2d flywheelToBot2d = botToFlywheel2d.inverse();

    public static double FuelWeight = 0.226; // kg
    public static double FlywheelRadius = 0.0762; // meters
    public static BaseFunction FlywheelEfficiency = new BaseConstant(() -> 0.5); // a function 1-to-1, TUNABLE NUM

    public static double GROUND_HEIGHT = 0.0;
    public static double HUB_HEIGHT = 1.8288;

    // TUNABLE (radian tuned)
    public static double RootFunctionTolerance = 0.005;
}
