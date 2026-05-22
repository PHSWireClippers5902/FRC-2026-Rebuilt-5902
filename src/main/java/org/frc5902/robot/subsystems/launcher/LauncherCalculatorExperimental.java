package org.frc5902.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import org.frc5902.robot.FieldConstants;
import org.frc5902.robot.RobotState;
import org.frc5902.robot.util.fieldbased.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class LauncherCalculatorExperimental {
    public static final InterpolatingDoubleTreeMap inch_velocity = new InterpolatingDoubleTreeMap();

    static {
        // anything under 58 NOTHING
        inch_velocity.put(Units.inchesToMeters(66.5), 155.0);
        inch_velocity.put(Units.inchesToMeters(79), 85.0);
        inch_velocity.put(Units.inchesToMeters(87.5), 165.0);
        inch_velocity.put(Units.inchesToMeters(102), 90.0);
        inch_velocity.put(Units.inchesToMeters(113.25), 175.0);
        inch_velocity.put(Units.inchesToMeters(128), 180.0);
        inch_velocity.put(Units.inchesToMeters(125), 185.0);
        inch_velocity.put(Units.inchesToMeters(146), 197.0);
        inch_velocity.put(Units.inchesToMeters(156), 207.0);
        inch_velocity.put(Units.inchesToMeters(168.5), 210.0);
        // anything over 167 NOTHING
    }

    public static double calculate() {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        Translation2d hubLocation = AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_LOCATION);
        // diff
        return calculate(currentPose.getTranslation().minus(hubLocation));
    }

    public static double calculate(Translation2d distance) {
        // filter
        if (distance.getDistance(new Translation2d()) < Units.inchesToMeters(60 + 16 + 23)) {
            return 0;
        }
        // plus magick 23in hub 16 in bot
        return inch_velocity.get(distance.getDistance(new Translation2d()) - Units.inchesToMeters(4));
    }

    public static boolean ready() {
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        double tolerance = 13.0;
        boolean returnval =
                (Math.abs((currentPose.getRotation().minus(calcPointedAngle()).getDegrees() % 360 + 360) % 360)
                        <= tolerance);
        // get new condition as well
        Logger.recordOutput("Outputs/MagickCalculator/ReturnValue", returnval);
        return returnval;
    }

    public static boolean readyWithSpeeds(double currentSpeed, double targetSpeed) {
        double tolerance = 20;
        return ready() && Math.abs(currentSpeed - targetSpeed) <= tolerance;
    }

    public static Rotation2d calcPointedAngle() {
        Rotation2d returnRot;
        Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
        Translation2d hubLocation = AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_LOCATION);
        returnRot = currentPose.getTranslation().minus(hubLocation).getAngle();
        Logger.recordOutput("Outputs/MagickCalcluator/CalcPointedAngle", returnRot);
        return returnRot;
    }
}
