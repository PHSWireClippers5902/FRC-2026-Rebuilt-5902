/**
 * @blame Daniel Sabalakov
 */
package org.frc5902.robot.util.flywheellib.flywheelfunctions;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import org.frc5902.robot.util.flywheellib.constants.FlywheelConstants;
import org.frc5902.robot.util.flywheellib.functions.BaseFunction;
import org.frc5902.robot.util.flywheellib.functions.QuadraticFunction;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.HashMap;

@ToString
public class FuelTimeFunction implements BaseFunction {
    @Getter @AutoLogOutput
    private double c;

    QuadraticFunction fuelTimeFunction =
            QuadraticFunction.builder().a(-9.8).build();

    BaseFunction verticalVelocityFunction = FuelVelocityOutOfLauncher.getFuelVelocityVerticalFunction();

    public FuelTimeFunction(double c) {
        fuelTimeFunction.setC(c);
    }
    public FuelTimeFunction() {
        fuelTimeFunction.setC(0.0);
    }

    public void setC(double newC) {
        this.c = newC;
        fuelTimeFunction.setC(this.c);
    }

    /**
     * Returns time,
     * @param input input as double angular velocity RPS
     * @return return greater root
     */
    @Override
    public double function(double angularVelocityRadiansPerSecond) {
        fuelTimeFunction.setB(verticalVelocityFunction.function(angularVelocityRadiansPerSecond));
        return fuelTimeFunction.getGreaterRoot();
    }
    /**
     * Return if the time function is a real number
     * @return is real
     */
    public boolean real() {
        return fuelTimeFunction.real();
    }

}
