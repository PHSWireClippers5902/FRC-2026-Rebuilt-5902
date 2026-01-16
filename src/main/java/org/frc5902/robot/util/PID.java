package org.frc5902.robot.util;

import lombok.Builder;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// We want built in get and set methods, a builder, tostring, a hashcode
// We also want a blank constructor
@Getter
@Setter
@Builder(builderMethodName = "")
@ToString
@EqualsAndHashCode
@NoArgsConstructor
public class PID implements LoggableInputs {

    public String pid_type;

    @Builder.Default
    public double proportional = 0;

    @Builder.Default
    public double integral = 0;

    @Builder.Default
    public double deriviative = 0;

    public PID(String pid_type, double proportional, double integral, double deriviative) {}

    public void fromLog(LogTable table) {
        this.proportional = table.get(pid_type + "_Proportional_val", 0);
        this.integral = table.get(pid_type + "_Integral_val", 0);
        this.deriviative = table.get(pid_type + "_Deriviative_val", 0);
    }

    /**
     *  Required methods
     */
    public void toLog(LogTable table) {
        table.put(pid_type + "_Proportional_val", this.proportional);
        table.put(pid_type + "_Integral_val", this.integral);
        table.put(pid_type + "_Deriviative_val", this.deriviative);
    }

    public static PIDBuilder builder(String pid_type) {
        return new PIDBuilder().pid_type(pid_type);
    }
}
