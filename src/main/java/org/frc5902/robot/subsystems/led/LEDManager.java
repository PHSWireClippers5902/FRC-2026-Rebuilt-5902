package org.frc5902.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import lombok.Getter;

import java.util.ArrayList;

public class LEDManager {

    private static LEDManager instance = null;
    private Spark ledcontroller;

    private ArrayList<ColorConstants.Color> colorqueue = new ArrayList<ColorConstants.Color>();

    @Getter
    private ColorConstants.Color CURRENT_COLOR = null;

    public static LEDManager getInstance() {
        if (instance == null) {
            instance = new LEDManager();
            return instance;
        }
        return instance;
    }

    // constructor
    private LEDManager() {
        ledcontroller = new Spark(9);
        colorqueue.add(getAllianceColor());
    }

    public void periodic() {
        calculateColor();
        ledcontroller.set(CURRENT_COLOR.getInput());
    }

    public void calculateColor() {
        this.CURRENT_COLOR = getAllianceColor();
    }

    public ColorConstants.Color getAllianceColor() {
        Alliance a = DriverStation.getAlliance().orElse(Alliance.Red);
        return a == Alliance.Red ? ColorConstants.SOLID_RED : ColorConstants.SOLID_BLUE;
    }
}
