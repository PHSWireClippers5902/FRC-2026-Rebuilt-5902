package org.frc5902.robot.util.fieldbased;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;

public abstract class Shape2d {
    public Alert[] alerts;

    public abstract double getArea();

    public abstract boolean insideShape(Translation2d pose);

    public abstract void transform(Translation2d by);
}
