package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

public class RobotConstants {

    public static double

        /*-- Localization --*/
        TICKS_PER_REV = 4096,
        WHEEL_RADIUS = 1.8898, // in

        parYTicks = 0.0, // y position of the parallel encoder (in tick units)
        perpXTicks = 0.0, // x position of the perpendicular encoder (in tick units)

        /*-- Robot Movement --*/
        maxVelocity = 0.0, // max target velocity for the path follow
        maxAcceleration = 0.0, // max target acceleration and deceleration for the path follow

        maxRotationalVelocity = 0.0, // max rotational target velocity for the path follow
        maxRotationalAcceleration = 0.0, // max rotational target acceleration and deceleration for the path follow

        minRadiusRange = 0.0, // min lookahead distance
        maxRadiusRange = 0.0, // max lookahead distance

        radiusMutliplier = 0.0, // multiplier of the dynamic radius

        robotX = 0.0, // robot's size in the x axis
        robotY = 0.0; // robot's size in the y axis

    /**
     * Calculates the inches per encoder tick
     * @return
     */
    public static double inPerTick_Calculation() {
        return 2 * WHEEL_RADIUS * Math.PI / TICKS_PER_REV;
    }
}
