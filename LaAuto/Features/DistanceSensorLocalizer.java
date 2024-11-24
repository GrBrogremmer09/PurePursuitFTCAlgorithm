package org.firstinspires.ftc.teamcode.LaAuto.Features;

import static java.lang.Math.cos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;

public class DistanceSensorLocalizer {

    private final int FIELD_SIZE = 144; // Inches

    private static final double
        SIDE_DIST_OFFSET = 0, // Inches
        REAR_DIST_OFFSET = 0; // Inches

    private double
        x_direction_mltplr = 1,
        y_direction_mltplr = 1;

    public Vector2d calculateRealLocation(Pose2d curr_pose,
                                          DistanceSensorEx rear_Dist,
                                          DistanceSensorEx side_Dist) {

        if (curr_pose.position.x < 0) x_direction_mltplr = -1;
        if (curr_pose.position.y < 0) y_direction_mltplr = -1;

        double real_X =
            (FIELD_SIZE / 2) - cos(90 - curr_pose.heading.toDouble()) *
                (side_Dist.getDistance(DistanceUnit.INCH) + SIDE_DIST_OFFSET);

        double real_Y =
            (FIELD_SIZE / 2) - cos(90 - curr_pose.heading.toDouble()) *
                (rear_Dist.getDistance(DistanceUnit.INCH) + REAR_DIST_OFFSET);

        return new Vector2d(x_direction_mltplr * real_X, y_direction_mltplr * real_Y);
    }
}