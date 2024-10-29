package org.firstinspires.ftc.teamcode.LaAuto.Features;

import static java.lang.Math.cos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;

public class DistanceSensorLocalizer {
    private DistanceSensorEx
        side_Dist,
        leftDist,
        rightDist,
        rear_Dist;

    private final int FIELD_SIZE = 144; // Inches

    private static final double
        SIDE_DIST_OFFSET = 0, // Inches
        REAR_DIST_OFFSET = 0; // Inches

    protected double
        x_direction_mltplr = 1,
        y_direction_mltplr = 1,
        real_X,
        real_Y;

    public DistanceSensorLocalizer (DistanceSensorEx rear_Dist,
                                    DistanceSensorEx left_Dist,
                                    DistanceSensorEx right_Dist) {
        this.rear_Dist = rear_Dist;
        this.leftDist = left_Dist;
        this.rightDist = right_Dist;
    }

    private void quadrant_masc(Pose2d pose) {
        if (pose.position.x < 0 && pose.position.y > 0) {
            x_direction_mltplr = -1;
            y_direction_mltplr = 1;
            side_Dist = leftDist;
        } else if (pose.position.x < 0 && pose.position.y < 0) {
            x_direction_mltplr = -1;
            y_direction_mltplr = -1;
            side_Dist = rightDist;
        } else if (pose.position.x > 0 && pose.position.y < 0) {
            x_direction_mltplr = 1;
            y_direction_mltplr = -1;
            side_Dist = leftDist;
        } else {
            x_direction_mltplr = 1;
            y_direction_mltplr = 1;
            side_Dist = rightDist;
        }
    }

    public Vector2d calculateRealLocation(Pose2d curr_pose) {
        quadrant_masc(curr_pose);

        real_X = (FIELD_SIZE/2) - cos(90 - curr_pose.heading.toDouble()) *
            (side_Dist.getDistance(DistanceUnit.INCH) + SIDE_DIST_OFFSET);
        real_Y= (FIELD_SIZE/2) - cos(90 - curr_pose.heading.toDouble()) *
            (rear_Dist.getDistance(DistanceUnit.INCH) + REAR_DIST_OFFSET);

        return new Vector2d(x_direction_mltplr * real_X, y_direction_mltplr * real_Y);
    }
}
