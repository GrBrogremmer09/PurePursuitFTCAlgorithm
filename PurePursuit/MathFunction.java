package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunction {

    /**
     * Makes sure the range of the angle is within the limits of 180, -180 degrees
     * @param angle
     * @return
     */
    public static double calculateAngleUnwrap(double angle) {
        while (angle < -180) {
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
    public static Point calculateCircleIntersection(Point state, double radius,
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
     * Stopping condition for the robot's position
     * @param currentPoint
     * @param targetPoint
     * @return
     */
    public static boolean positionEqualsThreshold(Point currentPoint, WayPoint targetPoint) {
        if (currentPoint.x <= targetPoint.pose[0] + targetPoint.threshold[0] && currentPoint.x >= targetPoint.pose[0] - targetPoint.threshold[0])
            if (currentPoint.y <= targetPoint.pose[1] + targetPoint.threshold[0] && currentPoint.y >= targetPoint.pose[1] - targetPoint.threshold[0])
                return true;
        return false;
    }

    /**
     * Stopping condition for the robot's rotation
     * @param currentHeading
     * @param targetPoint
     * @return
     */
    public static boolean rotationEqualsThreshold(double currentHeading, WayPoint targetPoint) {
        if (currentHeading <= targetPoint.pose[2] + targetPoint.threshold[1] && currentHeading >= targetPoint.pose[2] - targetPoint.threshold[1])
                return true;
        return false;
    }
}