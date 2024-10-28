package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxAcceleration;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRadiusRange;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRotationalAcceleration;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxRotationalVelocity;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.maxVelocity;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.minRadiusRange;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.radiusMutliplier;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.calculateAngleUnwrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.calculateCircleIntersection;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.positionEqualsThreshold;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.rotationEqualsThreshold;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.TwoDeadWheelLocalizer;
import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    public static TwoDeadWheelLocalizer localizer;
    public static ElapsedTime time;
    public static double previousTime;

    public static volatile Pose2d pose = new Pose2d(0,0,0);

    public static volatile double
        positionTargetVelocity,
        rotationalTargetVelocity,
        currentRadius;

    public static DcMotor
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor"),
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"),
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor"),
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

    public RobotMovement(HardwareMap hm) {
        localizer = new TwoDeadWheelLocalizer(hm);
        time = new ElapsedTime();

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * The logic and final function to run the pure pursuit algorithm
     * @param points
     */
    public static void followPath(ArrayList<WayPoint> points) {

        boolean isFinished = false;

        double[] motorsPower;

        WayPoint followPoint = new WayPoint(points.get(0));

        WayPoint start =
            new WayPoint.WaypointBuilder(new double[]{0,0,0}, WayPoint.WaypointType.DEFAULT).build();
        WayPoint end =
            new WayPoint.WaypointBuilder(new double[]{0,0,0}, WayPoint.WaypointType.DEFAULT).build();

        while(!isFinished) {

            pose = pose.plus(localizer.update().value());
            pose = new Pose2d(pose.component1(),
                              Math.toDegrees(pose.heading.toDouble()));
            currentRadius = calculateRadius(getCurrentVelocity());

            for(int i = points.size() - 1; i >= 0; --i) {
                start = points.get(i);
                end = points.get(i + 1);

                if (end.type == WayPoint.WaypointType.END) {
                    if (hypot(end.pose[0] - pose.position.x, end.pose[1] - pose.position.y) <= currentRadius) {
                        followPoint = end;
                        break;
                    }
                }

                Point currentPoint = calculateCircleIntersection(
                    new Point(pose.position.x, pose.position.y),
                    currentRadius,
                    new Point(start.pose[0], start.pose[1]),
                    new Point(end.pose[0], end.pose[1])
                );

                if (currentPoint != null){
                    followPoint.setPoint(currentPoint);
                    break;
                }
            }

            positionMotionProfiling(end);
            rotationMotionProfiling(end);

            motorsPower = goToPoint(followPoint.pose[0], followPoint.pose[1], followPoint.pose[2]);

            if (positionEqualsThreshold(new Point(pose.position.x, pose.position.y), end) &&
                rotationEqualsThreshold(pose.heading.toDouble(), end)) isFinished = true;

            robotCentricMovement(motorsPower[0], motorsPower[1], motorsPower[2]);
        }

        robotCentricMovement(0, 0, 0);
    }

    /**
     * @param x
     * @param y
     */
    public static double[] goToPoint(double x, double y, double preferredAngle) {
        double[] answer = new double[3];

        double distanceToTarget = Math.hypot(x - pose.position.x, y - pose.position.y);
        double targetAngle = Math.toDegrees(Math.atan2(y - pose.position.y, x - pose.position.x));
        double angleError = calculateAngleUnwrap(targetAngle - pose.heading.toDouble());

        double relativeXToPoint = Math.cos(angleError) * distanceToTarget;
        double relativeYToPoint = Math.sin(angleError) * distanceToTarget;

        double movementXPower = relativeXToPoint / distanceToTarget;
        double movementYPower = relativeYToPoint / distanceToTarget;

        double relativeTurnAngle = angleError - 180 + preferredAngle;
        // TODO: make sure relativeTurnAngle is between -180 and 180

        answer[0] = movementXPower * positionTargetVelocity;
        answer[1] = movementYPower * positionTargetVelocity;
        answer[2] = relativeTurnAngle/180 * rotationalTargetVelocity;

        return answer;
    }

    /**
     * Motion Profiling function for the position (robot movement)
     * @param endPoint
     */
    public static void positionMotionProfiling(WayPoint endPoint) {
        Vector2d robotPosition = pose.position;
        double currentVelocity = getCurrentVelocity();
        double currentTime = time.time();

        if (maxVelocity > currentVelocity) { // Acceleration condition
            positionTargetVelocity = (
                currentVelocity + maxAcceleration * (currentTime - previousTime)
            );
        } else {
            positionTargetVelocity = maxVelocity; // Cruising condition
        }

        if (endPoint.type == WayPoint.WaypointType.END) {
            double distance = Math.hypot(endPoint.pose[0] - robotPosition.x,
                                         endPoint.pose[1] - robotPosition.y);

            if (distance <= (Math.pow(positionTargetVelocity, 2) / (2 * maxAcceleration))) { // Decel condition
                positionTargetVelocity = (
                    currentVelocity - maxAcceleration * (currentTime - previousTime)
                );
            }
        }

        positionTargetVelocity = Range.clip(positionTargetVelocity, 0, 1);

        previousTime = currentTime;
    }

    /**
     * Motion Profiling function for the rotation (heading) of the robot
     * @param endPoint
     */
    public static void rotationMotionProfiling(WayPoint endPoint) {
        double currentVelocity = getCurrentRotationalVelocity();
        double currentTime = time.time();

        double distance = calculateAngleUnwrap(endPoint.pose[2] - pose.heading.toDouble());
        int directionMultiplier = 1;

        if (distance < 0) {
            directionMultiplier = -1;
        }

        if (maxRotationalVelocity > abs(currentVelocity)) { // Acceleration condition
            rotationalTargetVelocity = (
                currentVelocity + maxRotationalAcceleration * (currentTime - previousTime)
            );
        } else {
            rotationalTargetVelocity = maxRotationalVelocity; // Cruising condition
        }

        if (endPoint.type == WayPoint.WaypointType.END) {
            if (distance <= (Math.pow(rotationalTargetVelocity, 2) / (2 * maxRotationalAcceleration))) { // Decel condition
                rotationalTargetVelocity =
                    currentVelocity - directionMultiplier * maxRotationalAcceleration * (currentTime - previousTime);
            }
        }

        rotationalTargetVelocity = Range.clip(rotationalTargetVelocity, -1, 1);

        previousTime = currentTime;
    }

    /**
     * Calculates the dynamic radius
     * @param currentVelocity
     * @return radius
     */
    public static double calculateRadius(double currentVelocity) {
        double radius = minRadiusRange + (currentVelocity * radiusMutliplier);
        radius = Range.clip(radius, minRadiusRange, maxRadiusRange);

        return radius;
    }

    /**
     * Basic robot-centric movement
     */
    public static void robotCentricMovement(double x, double y, double t) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(t), 1);
        double frontLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double frontRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Robots velocity
     * @return getCurrentVelocity
     */
    public static double getCurrentVelocity() {
        return localizer.getCurrentVelocity();
    }

    /**
     * Robots rotational (heading) velocity
     * @return getCurrentRotationalVelocity
     */
    public static double getCurrentRotationalVelocity() {
        return localizer.getCurrentRotationalVelocity();
    }
}