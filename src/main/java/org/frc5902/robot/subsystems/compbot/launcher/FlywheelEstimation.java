package org.frc5902.robot.subsystems.compbot.launcher;

import org.frc5902.robot.util.flywheellib.flywheelfunctions.FlywheelRootEstimator2;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;
import lombok.Setter;

public class FlywheelEstimation {
    // create util object
    private final FlywheelRootEstimator2 root;
    private FlywheelEstimation instance = null;


    @Getter @Setter @AutoLogOutput
    public Goal goal;

    public FlywheelEstimation getInstance() {
        if (instance == null) {
            instance = new FlywheelEstimation();
        }
        return instance;
    }


    private FlywheelEstimation() {
        root = new FlywheelRootEstimator2();
    }

    public double getTotalFlywheelVelocity() {
        switch (goal) {
            case FLOOR_LEFT -> {
                return 0.0;
            }
            case FLOOR_RIGHT -> {
                return 0.0;
            }
            case HUB -> {
                return 0.0;
            }
            default -> {
                return 0.0;
            }
        }
    }

    public double getRobotOrientation() {
        switch (goal) {
            case FLOOR_LEFT -> {
                return 0.0;
            }
            case FLOOR_RIGHT -> {
                return 0.0;
            }
            case HUB -> {
                return 0.0;
            }
            default -> {
                return 0.0;
            }
        }
    }



    public enum Goal {
        FLOOR_LEFT,
        FLOOR_RIGHT,
        HUB
    }

}
