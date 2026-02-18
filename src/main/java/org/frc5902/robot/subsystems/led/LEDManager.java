package org.frc5902.robot.subsystems.led;

public class LEDManager {

    private static LEDManager instance = null;

    public static LEDManager getInstance() {
        if (instance == null) {
            instance = new LEDManager();
            return instance;
        }
        return instance;
    }

    // constructor
    private LEDManager() {}
}
