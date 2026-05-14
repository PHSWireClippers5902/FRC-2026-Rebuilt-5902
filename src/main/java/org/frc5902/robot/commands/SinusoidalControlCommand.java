/**
 * @team: The Wire Clippers 5902
 * @name:    Sinusoidals
 * @purpose: A place to simulate sinusoidal controls in an FRC match
 * @author:    Daniel Sabalakov
 */
package org.frc5902.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleConsumer;

public class SinusoidalControlCommand extends Command {
    private DoubleConsumer method;
    private Timer sinusoidalTimer;

    // PROPER sine function: f(x) = a * sin(x - h) + k
    // FOR seconds: f(t) = a * sin(2PI/period * (t-h)) + k
    @Getter
    @Setter
    private double magnitude = 1;

    @Getter
    @Setter
    private double period = 1;

    @Getter
    @Setter
    private double shift = 0.0;

    @Getter
    @Setter
    private double raise = 0.0;

    private BaseFunction sinefunction;

    @Setter
    @Getter
    private double finalValue = 0.0;

    @Getter
    private String name = "";

    @Getter 
    private double cycles = -1;

    public SinusoidalControlCommand(
            SubsystemBase system,
            DoubleConsumer method,
            String name,
            double magnitude,
            double period,
            double shift,
            double raise,
            double cycles,
            double finalValue,
            boolean useCosine) {
        this.method = method;
        sinusoidalTimer = new Timer();

        this.magnitude = magnitude;
        this.period = period;
        this.shift = shift;
        this.raise = raise;

        this.cycles = cycles;
        this.name = name;

        this.finalValue = finalValue;

        sinefunction = (value) -> this.raise
                + this.magnitude
                        * ((useCosine)
                                ? Math.cos(2 * Math.PI * (value - this.shift) / this.period)
                                : Math.sin(2 * Math.PI * (value - this.shift) / this.period));

        if (system != null) addRequirements(system);
    }

    @Override
    public void initialize() {
        sinusoidalTimer.reset();
        sinusoidalTimer.start();
    }

    @Override
    public void execute() {
        double result = sinefunction.function(sinusoidalTimer.get());
        method.accept(result);
        Logger.recordOutput("Sinusoidals/" + this.name, result);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) method.accept(finalValue);
    }

    @Override
    public boolean isFinished() {
        return sinusoidalTimer.get() / period > cycles;
    }

    public static SinusoidalControlCommand FakeSinusoidalCommand(String name) {
        return new SinusoidalControlCommand(
                null,
                (double value) -> {},
                "FakeSine/" + name,
                1,
                3,
                0,
                0,
                -1,
                0.0,
                true);
    }
}
