package org.usfirst.frc.team503.robot.lidar.icp;

import org.usfirst.frc.team503.lib.geometry.Translation2d;

public class Point {

    public final double x, y, angle;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = Math.atan2(y, x);
    }

    public Point(Translation2d t) {
        this(t.x(), t.y());
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    public double getDistanceSq(Point p) {
        double dx = x - p.x, dy = y - p.y;
        return dx * dx + dy * dy;
    }

    public double getDistance(Point p) {
        return Math.sqrt(getDistanceSq(p));
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    

}