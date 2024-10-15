package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

public class WayPoint {
    public enum WaypointType{
        END,
        DEFAULT
    }

    public double x;
    public double y;
    public double radius;
    public double moveSpeed;
    public double turnSpeed;
    public WaypointType type;

    public WayPoint(double x, double y, double radius,
                    double moveSpeed, double turnSpeed) {

        this.x = x;
        this.y = y;
        this.radius = radius;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
    }

    public WayPoint(WayPoint point) {
            x = point.x;
            y = point.y;
            radius = point.radius;
            moveSpeed = point.moveSpeed;
            turnSpeed = point.turnSpeed;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint (Point point) {
        x = point.x;
        y = point.y;
    }

    public void setType() {

    }
}
