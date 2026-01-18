package org.frc5902.robot.util;

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

    @Builder.Default
    public double proportional = 0;

    @Builder.Default
    public double integral = 0;

    @Builder.Default
    public double deriviative = 0;

    public PID(double proportional, double integral, double deriviative) {}

    public static PIDBuilder builder() {
        return new PIDBuilder();
    }
}
