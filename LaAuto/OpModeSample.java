package org.firstinspires.ftc.teamcode.LaAuto;

import static org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions.poseAdjuster;
import static org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.RobotMovement.pose;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LaAuto.Features.BuilderFunctions;
import org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.LaAuto.PurePursuit.WayPoint;

import java.util.ArrayList;

@Autonomous(name = "OpModeSample")
public class OpModeSample extends LinearOpMode {

//    private DistanceSensorLocalizer dist_localizer;
//
//    private DistanceSensorEx
//        rear_dist,
//        right_dist,
//        left_dist;

    public RobotMovement rm;

    private ArrayList <WayPoint> scenario_1 = new ArrayList <>();

    private WayPoint
        startingPose = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[]{
                0,0,0
            }, BuilderFunctions.RobotSides.CENTER
            ), WayPoint.WaypointType.DEFAULT
        ).build(),

        submersible = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                2 * Tile, 0, 0
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.DEFAULT
        ).build(),

        midPoint = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                2 * Tile, 2 * Tile, 90
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.END
        ).build();

    @Override
    public void runOpMode() {
        rm = new RobotMovement(hardwareMap);
//        dist_localizer = new DistanceSensorLocalizer(rear_dist, left_dist, right_dist);

        scenario_1.add(startingPose);
        scenario_1.add(submersible);
        scenario_1.add(midPoint);

        pose = new Pose2d(startingPose.pose[0], startingPose.pose[1], startingPose.pose[2]);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            rm.followPath(scenario_1);
            //        pose = new Pose2d(dist_localizer.calculateRealLocation(pose), pose.heading.toDouble());
            //        followPath(scenario_1);
        }
    }
}
