package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunction {

    /**
     * Makes sure the range of the angle is within the limits of 180, -180 degrees
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle) {
        while (angle <  -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }

        return angle;
    }

    /**
     * Finds the x, y coordinates of the intersections between the circle and the current segment
     * @param state
     * @param radius
     * @param start
     * @param end
     * @return
     */
    public static Point CircleIntersections(Point state, double radius,
                                                        Point start, Point end) {

        double dx = end.x - start.x;
        double dy = end.y - start.y;

        // Calculate the differences in x, y between the robot's position and the segment's start
        double fx = start.x - state.x;
        double fy = start.y - state.y;

        // Coefficients for the quadratic equation (a*t^2 + b*t + c = 0)
        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - radius * radius;

        // Calculate the discriminant to check if the circle intersects with the segment
        double discriminant = b * b - 4 * a * c;

        // If discriminant is negative, no intersection; emergency route
        if (discriminant < 0) {
            //TODO: make emergency route re-localisation
            return null;
        } else {
            // Calculate the two possible values of t where intersections occur
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / (2 * a);
            double t2 = (-b - discriminant) / (2 * a);
            double t = Math.max(t1, t2);

            if (t < 0.0 || t > 1.0){
                //TODO: make emergency route re-localisation
            }

            double lx = start.x + t * dx;
            double ly = start.y + t * dy;

            return new Point(lx, ly);
        }
    }

    /**
     *
     * @param pathPoints
     * @param robotLocation
     * @param radius
     * @return
     */
    public static WayPoint getFollowPathWayPoint(ArrayList<WayPoint> pathPoints,
                                                 Point robotLocation, double radius) {
        WayPoint followPoint = new WayPoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; ++i) {
            WayPoint start = pathPoints.get(i);
            WayPoint end = pathPoints.get(i + 1);

            Point intersection = CircleIntersections(
                robotLocation, radius,
                start.toPoint(), end.toPoint()
            );

            followPoint.setPoint(intersection);

        }

        return followPoint;
    }
}