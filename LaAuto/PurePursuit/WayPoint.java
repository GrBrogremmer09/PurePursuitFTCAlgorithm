package org.firstinspires.ftc.teamcode.LaAuto.PurePursuit;

import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.maxAcceleration;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.maxRadiusRange;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.maxRotationalAcceleration;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.maxRotationalVelocity;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.maxVelocity;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.HardwareRelated.RobotConstants.minRadiusRange;

import org.opencv.core.Point;

public class WayPoint {

    public enum WaypointType{END, DEFAULT}

    public double[] pose = new double[3];
    public double[] threshold = new double[2];
    public double
        min_radius = minRadiusRange,
        max_radius = maxRadiusRange,
        max_vel = maxVelocity,
        max_accel = maxAcceleration,
        max_rot_vel = maxRotationalVelocity,
        max_rot_accel = maxRotationalAcceleration;

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
        this.min_radius = builder.min_radius;
        this.max_radius = builder.max_radius;
        this.max_vel = builder.max_vel;
        this.max_accel = builder.max_accel;
        this.max_rot_vel = builder.max_rot_vel;
        this.max_rot_accel = builder.max_rot_accel;
    }

    public WayPoint(WayPoint point) {
        pose[0] = point.pose[0];
        pose[1] = point.pose[1];
        type = point.type;
        pose[2] = point.pose[2];
        threshold = point.threshold;
        min_radius = point.min_radius;
        max_radius = point.max_radius;
        max_vel = point.max_vel;
        max_accel = point.max_accel;
        max_rot_vel = point.max_rot_vel;
        max_rot_accel = point.max_rot_accel;
    }

    public Point toPoint() {
        return new Point(pose[0], pose[1]);
    }

    public void setPoint (Point point) {
        pose[0] = point.x;
        pose[1] = point.y;
    }
}
