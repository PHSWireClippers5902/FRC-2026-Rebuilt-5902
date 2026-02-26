package org.frc5902.robot.util.fieldbased;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@ToString
public class Rectangle2d extends Shape2d {
    @Getter
    @Setter
    // should be backleft
    private Translation2d corner1;

    @Getter
    @Setter
    // should be frontright
    private Translation2d corner2;

    public Rectangle2d(Translation2d corner1, Translation2d corner2) {
        this.corner1 = corner1;
        this.corner2 = corner2;
        this.alerts = new Alert[] {
            new Alert("Rectangle2d X's are the same: " + this, AlertType.kWarning),
            new Alert("Rectangle2d Y's are the same: " + this, AlertType.kWarning)
        };
        ensureCorners();
        checkRectangle();
    }

    @Override
    public double getArea() {
        // Rect angle formula: b  *  w
        return Math.abs(corner1.getX() - corner2.getX()) * Math.abs(corner1.getY() - corner2.getY());
    }

    private void checkRectangle() {
        alerts[0].set(corner1.getX() == corner2.getX());
        alerts[1].set(corner1.getY() == corner2.getY());
    }

    /**
     * Why do we need this method?
     * This is because we need to change corner1 to corner2 if it represents the frontright pose
     */
    private void ensureCorners() {
        // swap corner1 x and corner2 x if corner1 is greater than corner2
        if (corner1.getX() > corner2.getX()) {
            double corner1x = corner1.getX();
            corner1 = new Translation2d(corner2.getX(), corner1.getY());
            corner2 = new Translation2d(corner1x, corner2.getY());
        }
        // swap corner2 y and corner2 y if corner1 is greater than corner2
        if (corner2.getY() > corner2.getY()) {
            double corner1y = corner1.getY();
            corner1 = new Translation2d(corner1.getX(), corner2.getY());
            corner2 = new Translation2d(corner2.getX(), corner1y);
        }
    }

    @Override
    public boolean insideShape(Translation2d translation) {
        // return if the pose is within
        return (translation.getX() < corner1.getX() || translation.getX() > corner2.getX())
                || (translation.getY() < corner1.getY() || translation.getY() > corner2.getY());
    }

    @Override
    public void transform(Translation2d by) {
        this.corner1 = new Translation2d(this.corner1.getX() - by.getX(), this.corner1.getY() - by.getY());
        this.corner2 = new Translation2d(this.corner2.getX() - by.getX(), this.corner2.getY() - by.getY());
    }
}
