package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.opencv.core.Point;

public class WayPoint {
    public enum WaypointType{
        END,
        DEFAULT,
    }

    public double x, y, t, threshold;
    public WaypointType type;

    public static class WaypointBuilder {
        private double x, y, t, threshold, min_radius, max_radius,
                        max_vel, max_accel, max_rot_vel, max_rot_accel;

        private WaypointType type;

        public WaypointBuilder(double x, double y, double t, WaypointType type) {
            this.x = x;
            this.y = y;
            this.t = t;
            this.type = type;
        }

        public WaypointBuilder threshold(double threshold) {
            this.threshold = threshold;
            return this;
        }

        public WaypointBuilder min_radius(double min_radius) {
            this.min_radius = min_radius;
            return this;
        }

        public WaypointBuilder max_radius(double max_radius) {
            this.max_radius = max_radius;
            return this;
        }

        public WaypointBuilder max_vel(double max_vel) {
            this.max_vel = max_vel;
            return this;
        }

        public WaypointBuilder max_accel(double max_accel) {
            this.max_accel = max_accel;
            return this;
        }

        public WaypointBuilder max_rot_vel(double max_rot_vel) {
            this.max_rot_vel = max_rot_vel;
            return this;
        }

        public WaypointBuilder max_rot_accel(double max_rot_accel) {
            this.max_rot_accel = max_rot_accel;
            return this;
        }

        public WayPoint build() {
            return new WayPoint(this);
        }
    }

    private WayPoint(WaypointBuilder builder) {
        this.x = builder.x;
        this.y = builder.y;
        this.type = builder.type;
        this.t = builder.t;
        this.threshold = builder.threshold;
        RobotConstants.minRadiusRange = builder.min_radius;
        RobotConstants.maxRadiusRange = builder.max_radius;
        RobotConstants.maxVelocity = builder.max_vel;
        RobotConstants.maxAcceleration = builder.max_accel;
        RobotConstants.maxRotationalVelocity = builder.max_rot_vel;
        RobotConstants.maxRotationalAcceleration = builder.max_rot_accel;
    }

    public WayPoint(WayPoint point) {
        x = point.x;
        y = point.y;
        type = point.type;
        t = point.t;
        threshold = point.threshold;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint (Point point) {
        x = point.x;
        y = point.y;
    }
}
