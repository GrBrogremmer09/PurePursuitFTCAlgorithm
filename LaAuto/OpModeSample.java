package org.firstinspires.ftc.teamcode.LaAuto;

import static org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions.poseAdjuster;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.RobotMovement.followPath;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.RobotMovement.pose;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions;
import org.firstinspires.ftc.teamcode.LaAuto.Features.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.WayPoint;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;

import java.util.ArrayList;

@Autonomous(name = "OpModeSample", group = "Auto")
public class OpModeSample extends LinearOpMode {

    private DistanceSensorLocalizer dist_localizer;

    private DistanceSensorEx
        rear_dist,
        right_dist,
        left_dist;

    private ArrayList <WayPoint> scenario_1 = new ArrayList <>();

    private WayPoint
        startingPose = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[]{
                2.5 * Tile, 3 * Tile, 90
            }, BuilderFunctions.RobotSides.REAR
            ), WayPoint.WaypointType.DEFAULT
        ).build(),

        submersible = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                3 * Tile, 1.5 * Tile, 45
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.DEFAULT
        ).threshold(new double[]{45, 89}).build(),

        midPoint = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                3 * Tile, 1.5 * Tile, 45
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.DEFAULT
        ).max_radius(5).build();

    @Override
    public void runOpMode() {
        new RobotMovement(hardwareMap);
        dist_localizer = new DistanceSensorLocalizer(rear_dist, left_dist, right_dist);

        scenario_1.add(startingPose);
        scenario_1.add(submersible);

        pose = new Pose2d(startingPose.pose[0], startingPose.pose[1], startingPose.pose[2]);

        waitForStart();

        followPath(scenario_1);
        pose = new Pose2d(dist_localizer.calculateRealLocation(pose), pose.heading.toDouble());
        followPath(scenario_1);
    }
}
