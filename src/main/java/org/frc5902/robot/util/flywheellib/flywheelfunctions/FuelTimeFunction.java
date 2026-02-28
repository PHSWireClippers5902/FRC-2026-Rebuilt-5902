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

import java.util.HashMap;

@ToString
public class FuelTimeFunction implements BaseFunction {

    @Getter
    @Setter
    private FuelTimeAim aim = FuelTimeAim.HUB;

    HashMap<FuelTimeAim, Double> aimMap = FlywheelConstants.aimToDifference;

    QuadraticFunction fuelTimeFunction =
            QuadraticFunction.builder().a(-9.8).c(aimMap.getOrDefault(aim, 0.0) - FlywheelConstants.botToFlywheel.getZ()).build();

    BaseFunction verticalVelocityFunction = FuelVelocityOutOfLauncher.getFuelVelocityVerticalFunction();

    public FuelTimeFunction() {}

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

    /**
     * Enumerator defined as GROUND, HUB (aiming passing locations)
     */
    public enum FuelTimeAim {
        GROUND,
        HUB
    }
}
