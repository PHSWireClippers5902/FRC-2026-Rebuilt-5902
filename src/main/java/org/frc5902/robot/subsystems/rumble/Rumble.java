package org.frc5902.robot.subsystems.rumble;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

public class Rumble {
    @Setter
    private CommandXboxController controller;

    @Getter
    @Setter
    @AutoLogOutput
    private RumblePriorities rumbleGoal = RumblePriorities.OFF;

    private static Rumble instance = null;

    public static Rumble getInstance(Optional<CommandXboxController> controller) {
        if (instance == null) {
            instance = new Rumble(controller);
        }
        return instance;
    }

    public static Rumble getInstance() {
        if (instance == null) {
            instance = new Rumble();
        }
        return instance;
    }

    private Rumble(Optional<CommandXboxController> controller) {
        this.controller = controller.get();
    }

    private Rumble() {
        this.controller = null;
    }

    private RumblePriorities lastPriority = null;

    public void periodic() {
        if (this.controller == null) {
            return;
        }
        // if the priority has changes
        if (lastPriority != null && lastPriority != rumbleGoal) {
            this.controller.setRumble(RumbleType.kBothRumble, 0.0);
        }
        // now, set the rumble motor
        this.controller.setRumble(rumbleGoal.getRumbleType(), rumbleGoal.getIntensity());
        lastPriority = rumbleGoal;
    }

    public void RUMBLE_OFF_FORCED() {
        if (this.controller == null) {
            return;
        }
        this.controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

    public enum RumblePriorities {
        LEFT(1.0, RumbleType.kLeftRumble),
        RIGHT(1.0, RumbleType.kRightRumble),
        BOTH_FULL(1.0, RumbleType.kBothRumble),
        OFF(0.0, RumbleType.kBothRumble);

        @Getter
        private RumbleType rumbleType;

        @Getter
        private double intensity;

        private RumblePriorities(double intensity, RumbleType rumbleType) {
            this.intensity = intensity;
            this.rumbleType = rumbleType;
        }
    }
}
