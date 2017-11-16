package org.firstinspires.ftc.teamcode;

public class RobotMap {
    //    public static final float GEAR_RATIO = 50;
    public static final float TICKS_PER_ROTATION = 415;
    public static final float WHEEL_DIAMETER = 4; //inch

    public static final float TURN_TOLERANCE = 2;
    public static final float P_TURN = 0.01f;

    public static final float DRIVE_TOLERANCE = 1;
    public static final float P_CONSTANT_DRIVING = .1f;

    public static double INCREMENT = .05;

    //value in inches
    public static float distanceToTicks(float dist){
        float circumference = (float)Math.PI* WHEEL_DIAMETER;
        float wheelRevolutions = dist/circumference;
//        float shaftRevolutions = wheelRevolutions ;
        return wheelRevolutions * TICKS_PER_ROTATION;
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
