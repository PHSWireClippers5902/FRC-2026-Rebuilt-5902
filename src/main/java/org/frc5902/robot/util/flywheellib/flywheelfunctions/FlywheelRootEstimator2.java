/**
 * Flywheel Root Estimator takes all of the previously defines functions and estimates the angle at which the robot must point.
 * There is only **one** solution for each equation.  
 * @todo refactor
 * @blame Daniel Sabalakov
 */
package org.frc5902.robot.util.flywheellib.flywheelfunctions;

import org.frc5902.robot.state.RobotState;
import org.frc5902.robot.util.flywheellib.constants.FlywheelConstants;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;
import org.frc5902.robot.util.flywheellib.mathutil.BisectionMethod;
import org.frc5902.robot.util.flywheellib.mathutil.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Flywheel Root Estimator takes all of the previously defines functions and estimates the angle at which the robot must point.
 * There is only **one** solution for each equation.  
 */
public class FlywheelRootEstimator2 {
    // Define a time function
    public static FuelTimeFunction timeFunc;
    public static RobotState robotState;
    /**
     * create an instance of the class. 
     * @todo implement either a single instance state. 
     */
    public FlywheelRootEstimator2(){
        timeFunc = new FuelTimeFunction();
        robotState = RobotState.getInstance();
    }
    // root = 0
    public Point CalculateFlywheelRoot() {
        // calculate lower and upper bounds
        double[] bounds = new double[]{10,1000};

        BaseFunction dfunc = FuelDistanceFunctions2.getRootFunction(timeFunc);
        double root = BisectionMethod.calculate(dfunc, bounds, FlywheelConstants.RootFunctionTolerance);
        
        double t = timeFunc.function(root);

        // get pose
        Pose2d estimatedPose = robotState.getEstimatedPose();
        Twist2d estimatedTwist = robotState.getFieldVelocity().toTwist2d(0.5);

        double x = estimatedPose.getX() - estimatedTwist.dx * t;
        double y = estimatedPose.getY() - estimatedTwist.dy * t;

        // Compute angle safely
        double theta = Math.atan2(y, x);

        return new Point(root, theta);
    }

    /**
     * Check if conditions are met given an angular velocity
     * @param angularVelocityRadiansPerSecond angular velocity in rads/sec to pass in
     * @return whether or not the function actually will work. Checks to make sure that the timeFunction is real, makes sure it gives a positive time, and make sure that in the X and Y direction the inverse sine and cosine functions are fine.
     */
    public static boolean conditionsMet(double angularVelocityRadiansPerSecond) {        
        // If the quadratic is not real, return false.
        if (!timeFunc.real()) return false;
        // if the time it takes for the flywheel to fly is less than 0, return false.
        if (timeFunc.function(angularVelocityRadiansPerSecond) < 0) return false;
        return true;
    }

}
