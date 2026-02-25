/**
 * @blame Daniel Sabalakov
 */
package org.frc5902.robot.util.flywheellib.functions;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.util.function.DoubleSupplier;

@ToString
public class BaseConstant implements BaseFunction {
    @Getter
    @Setter
    private DoubleSupplier constant;

    public BaseConstant(DoubleSupplier constant) {
        this.constant = constant;
    }

    @Override
    public double function(double input) {
        return constant.getAsDouble();
    }
}
