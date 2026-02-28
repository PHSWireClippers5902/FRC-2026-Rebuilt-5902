/**
 * @blame Daniel Sabalakov
 */
package org.frc5902.robot.util.flywheellib.flywheelfunctions;

import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.util.flywheellib.constants.FlywheelConstants;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;
// TODO IMPLEMENT POSE3D IN REAL ROBOT PROJECT... USE ROBOTSTATE

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;

public class FuelDistanceFunctions2 {
    /**
     * Return the root function (defined as f(x) - h(x))
     * @param t time function to avoid redundancy
     * @return Difference between the x and y functions
     */
    public static BaseFunction getRootFunction(FuelTimeFunction t, Pose3d aim) {
        return new BaseFunction() {
            @Override
            public double function(double omega) {
                RobotState robotState =  RobotState.getInstance();
                // Time of flight for this angular velocity
                t.setC(aim.getZ() - FlywheelConstants.botToFlywheel.getZ());
                double time = t.function(omega);

                if (time <= 0 || Double.isNaN(time)) return Double.NaN;

                // Launch speed from flywheel
                double v = FuelVelocityOutOfLauncher.getFuelVelocityDistanceFunction()
                        .function(omega);

                // Left side: projectile distance squared
                double projectileDistSq = (v * time) * (v * time);

                Pose2d estimatedPose = robotState.getEstimatedPose();
                Twist2d estimatedTwist = robotState.getFieldVelocity().toTwist2d(0.5);

                double x = estimatedPose.transformBy(FlywheelConstants.botToFlywheel2d).getX() - aim.getX() - estimatedTwist.dx * time;
                double y = estimatedPose.transformBy(FlywheelConstants.botToFlywheel2d).getY() - aim.getY() - estimatedTwist.dy * time;

                // Right side: intercept distance squared
                double targetDistSq = x * x + y * y;

                return projectileDistSq - targetDistSq;
            }
        };
    }
}
