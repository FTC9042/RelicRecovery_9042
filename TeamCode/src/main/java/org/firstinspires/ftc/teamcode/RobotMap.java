package org.firstinspires.ftc.teamcode;

public class RobotMap {
    public static final float GEAR_RATIO = 50;
    public static final float TICKS_PER_ROTATION = 831;
    public static final float TICKS_PER_INCH = 110;
    public static final float WHEEL_DIAMETER = 4; //inch

    public static final float TURN_TOLERANCE = 6f; //offset due to P only control
    public static final float P_TURN = 0.01f;

    public static final float DRIVE_TOLERANCE = 1;
    public static final float DRIVE_OFFSET = 400;
    public static final double P_CONSTANT_DRIVING = 0.01;

    public static final double circumference = Math.PI* WHEEL_DIAMETER;

    public static double INCREMENT = .05;

    //value in inches

    public static float distanceToTicks(float dist){
        double wheelRevolutions = dist/circumference;
        double shaftRevolutions = wheelRevolutions * GEAR_RATIO;
        return (float) (shaftRevolutions * TICKS_PER_ROTATION);
    }

    public static double smoothSpeed(double currentSpeed, double oldSpeed, double increment) {
        double finalSpeed; //changed speed
        if (currentSpeed < oldSpeed - increment) {
            finalSpeed = oldSpeed - increment;
        } else if (currentSpeed > oldSpeed + increment) {
            finalSpeed = oldSpeed + increment;
        } else {
            finalSpeed = currentSpeed;
        }
        return finalSpeed;
    }

}
