package org.frc5902.robot.subsystems.led;

import lombok.AllArgsConstructor;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
@EqualsAndHashCode
@AllArgsConstructor
public class ColorCombo {
    public String color;
    public double pwmInput;
}
