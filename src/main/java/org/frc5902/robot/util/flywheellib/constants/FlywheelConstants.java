/**
 * Flywheel Constants are to be tuned in the robot.
 * @blame Daniel Sabalakov
 * @todo Confirm measurements, integrate into wpilib by making a sample robot project.
 */
package org.frc5902.robot.util.flywheellib.constants;

import org.frc5902.robot.util.flywheellib.flywheelfunctions.FuelTimeFunction.FuelTimeAim;
import org.frc5902.robot.util.flywheellib.functions.BaseConstant;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class FlywheelConstants {
    public static double FlywheelMountedAngleRadians = Math.PI / 6;
    // 7.5, 8, 17, from BACK RIGHT
    // length robot is 27
    public static Transform3d botToFlywheel = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5 - 7.5),
            Units.inchesToMeters(13.5 + 8),
            Units.inchesToMeters(17)
        ),
        new Rotation3d(0,0,-Math.PI/2)
    ); 
    public static Transform3d flywheelToBot = botToFlywheel.inverse();
    public static double FuelWeight = 0.226; // kg
    public static double FlywheelRadius = 0.0762; // meters
    public static BaseFunction FlywheelEfficiency = new BaseConstant(() -> 0.5); // a function 1-to-1, TUNABLE NUM
    public static HashMap<FuelTimeAim, Double> aimToDifference;

    static {
        aimToDifference = new HashMap<FuelTimeAim, Double>();
        aimToDifference.put(FuelTimeAim.GROUND, -0.3);
        aimToDifference.put(FuelTimeAim.HUB, 1.8288);
    }
    // TUNABLE (radian tuned)
    public static double RootFunctionTolerance = 0.005;

    // TUN
    public static DoubleSupplier dx = () -> -4;
    public static DoubleSupplier vx = () -> 0;
    public static DoubleSupplier dy = () -> -4;
    public static DoubleSupplier vy = () -> 0;
}
