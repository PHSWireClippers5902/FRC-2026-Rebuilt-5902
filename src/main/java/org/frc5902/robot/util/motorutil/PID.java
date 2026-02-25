package org.frc5902.robot.util.motorutil;

import lombok.Builder;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;

// We want built in get and set methods, a builder, tostring, a hashcode
// We also want a blank constructor
@Getter
@Setter
@Builder
@ToString
@EqualsAndHashCode
@NoArgsConstructor
public class PID {

    public double proportional = 0;
    public double integral = 0;

    public double deriviative = 0;

    @Builder
    public PID(double proportional, double integral, double deriviative) {
        this.proportional = proportional;
        this.integral = integral;
        this.deriviative = deriviative;
    }

    public static PIDBuilder builder() {
        return new PIDBuilder();
    }
}
