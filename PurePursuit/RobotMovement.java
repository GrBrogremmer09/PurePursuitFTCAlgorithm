package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants.inPerTick_Calculation;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.AngleWrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunction.getFollowPathWayPoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.TwoDeadWheelLocalizer;
import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    public static TwoDeadWheelLocalizer localizer;
    public static Pose2d pose;

    public RobotMovement(HardwareMap hm){
        localizer = new TwoDeadWheelLocalizer(hm, inPerTick_Calculation());
    }

    public static void followPath(ArrayList<WayPoint> points, double prefferedAngle) {
        WayPoint followPoint = getFollowPathWayPoint(points,
                                                     new Point(pose.position.x, pose.position.y),
                                                     current_radius
        );

        goToPoint(followPoint.x, followPoint.y, followPoint.moveSpeed,
                  prefferedAngle, followPoint.turnSpeed
        );
    }

    /**
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPoint(double x, double y, double movementSpeed, double preferredAngle,
                                 double turnSpeed) {

        pose = pose.plus(localizer.update().value());

        double distanceToTarget = Math.hypot(x - pose.position.x, y - pose.position.y);
        double absoluteAngleToTarget = Math.toDegrees(Math.atan2(y - pose.position.y, x - pose.position.x));
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (pose.heading.toDouble() - 90));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - 180 + preferredAngle;

        robotCentricMovement(movementXPower * movementSpeed,
                             movementYPower * movementSpeed,
                             Range.clip(relativeTurnAngle/30, -1, 1) * turnSpeed
        );
    }

    /**
     * Basic robot-centric movement
     */
    public static void robotCentricMovement(double x, double y, double t) {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        double frontLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double frontRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}