package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.opencv.core.Point;

public class WayPoint {

    enum WaypointType{END, DEFAULT}

    public double[] pose = new double[3];
    public double[] threshold = new double[2];

    public WaypointType type;

    public static class WaypointBuilder {
        private double
            min_radius, max_radius,
            max_vel, max_accel, max_rot_vel, max_rot_accel;

        private double[] threshold = new double[2];
        private double[] pose = new double[3];

        private WaypointType type;

        public WaypointBuilder(double[] pose, WaypointType type) {
            this.pose[0] = pose[0];
            this.pose[1] = pose[1];
            this.pose[2] = pose[2];
            this.type = type;
        }

        public WaypointBuilder threshold(double[] threshold) {
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
        this.pose[0] = builder.pose[0];
        this.pose[1] = builder.pose[1];
        this.type = builder.type;
        this.pose[2] = builder.pose[2];
        this.threshold = builder.threshold;
        RobotConstants.minRadiusRange = builder.min_radius;
        RobotConstants.maxRadiusRange = builder.max_radius;
        RobotConstants.maxVelocity = builder.max_vel;
        RobotConstants.maxAcceleration = builder.max_accel;
        RobotConstants.maxRotationalVelocity = builder.max_rot_vel;
        RobotConstants.maxRotationalAcceleration = builder.max_rot_accel;
    }

    public WayPoint(WayPoint point) {
        pose[0] = point.pose[0];
        pose[1] = point.pose[1];
        type = point.type;
        pose[2] = point.pose[2];
        threshold = point.threshold;
    }

    public Point toPoint() {
        return new Point(pose[0], pose[1]);
    }

    public void setPoint (Point point) {
        pose[0] = point.x;
        pose[1] = point.y;
    }
}
