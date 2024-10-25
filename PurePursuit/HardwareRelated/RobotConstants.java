package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

public class RobotConstants {

    public static final double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 1.8898; // in

    public static double inPerTick_Calculation() {
        return 2 * WHEEL_RADIUS * Math.PI / TICKS_PER_REV;
    }

    public static double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
    public static double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)

    public static double maxVelocity = 0; // max target velocity for the path follow
    public static double maxAcceleration = 0; // max target acceleration and deceleration for the path follow

    public static double maxRotationalVelocity = 0; // max rotational target velocity for the path follow
    public static double maxRotationalAcceleration = 0; // max rotational target acceleration and deceleration for the path follow

    public static double minRadiusRange = 0; // min lookahead distance
    public static double maxRadiusRange = 0; // max lookahead distance

    public static double radiusMutliplier = 0; // multiplier of the dynamic radius
}
