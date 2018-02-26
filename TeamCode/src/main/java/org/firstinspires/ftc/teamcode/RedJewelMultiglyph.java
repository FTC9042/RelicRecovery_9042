package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by anikaitsingh on 2/24/18.
 */
@Autonomous(name = "Red Jewel - Multi-Glyph", group = "Auton")
public class RedJewelMultiglyph extends LinearOpMode{
    Robot robot;
    ColorSensor sensorColor;
    double errorLeft, errorRight;
    double positionLeftAverage, positionRightAverage;
    BNO055IMU imu;

    //PID Objects
    PID pDrivingLeft = new PID(RobotMap.P_CONSTANT_DRIVING);
    PID pDrivingRight = new PID(RobotMap.P_CONSTANT_DRIVING);

    RelicRecoveryVuMark position = RelicRecoveryVuMark.UNKNOWN;

    private void initR() {
        robot = new Robot(this.hardwareMap);
        HardwareMap map = this.hardwareMap;
        robot.leftFront = map.dcMotor.get("1");
        robot.leftBack = map.dcMotor.get("2"); // changed originally rightFront
        robot.rightFront = map.dcMotor.get("3");
        robot.rightBack = map.dcMotor.get("4");

        robot.jewel = map.servo.get("jewelS");
        robot.flipper = map.servo.get("Flipper");
        robot.color = map.colorSensor.get("color");
        robot.intakeLeft = map.dcMotor.get("intakeLeft");
        robot.intakeRight = map.dcMotor.get("intakeRight");

        robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runOpMode(){
        initR();

        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorColor.enableLed(true);
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        waitForStart();

        boolean red = false;

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        String str = "init";

        while (opModeIsActive()) {
            robot.jewel.setPosition(0.3);

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR), (int) (sensorColor.green() * SCALE_FACTOR), (int) (sensorColor.blue() * SCALE_FACTOR), hsvValues);

            red = hsvValues[0] < 30|| hsvValues[0] > 330;
            if (red) str = "red";
            else str = "not red";


            // send the info back to driver station using telemetry function.
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Is Red", str);
            telemetry.addData("Servo Position", robot.jewel.getPosition());
            telemetry.update();

            if (time.seconds() > 3) break;
        }


        telemetry.addData("Detected", str);
        telemetry.update();

        double power;
        if (red) {
            power = -.2;
        } else {
            power = .2;
        }

        //for testing purposes
        time.reset();
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            robot.setDrivePower(power);
            if (time.time() > 0.5) {
                break;
            }
            telemetry.addData("Detected", str);
            telemetry.update();
        }
        robot.jewel.setPosition(0.9);


        robot.stop();

        if(!red){
            power = -.2;
        }

        time.reset();
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            robot.setDrivePower(power);
            if (time.time() > 1.0) {
                break;
            }
            telemetry.update();
        }

        robot.stop();

        // *****************************************************************************************
        // Driving Distance
        // *****************************************************************************************

        //setting target value
        double targetInches = 25.5;

        pDrivingLeft.setTarget(robot.leftFront.getCurrentPosition() + targetInches * RobotMap.TICKS_PER_INCH);//25 inches
        pDrivingRight.setTarget(robot.rightFront.getCurrentPosition() + targetInches * RobotMap.TICKS_PER_INCH);//25 inches

        errorLeft = targetInches* RobotMap.TICKS_PER_INCH - robot.leftFront.getCurrentPosition();
        errorRight = targetInches* RobotMap.TICKS_PER_INCH - robot.rightFront.getCurrentPosition();

        //Proportional Loop
        while(opModeIsActive() && errorLeft > RobotMap.DRIVE_TOLERANCE && errorRight > RobotMap.DRIVE_TOLERANCE){
            positionLeftAverage = robot.leftFront.getCurrentPosition();
            positionRightAverage = robot.rightFront.getCurrentPosition();

            errorLeft = targetInches* RobotMap.TICKS_PER_INCH - robot.leftFront.getCurrentPosition();
            errorRight = targetInches* RobotMap.TICKS_PER_INCH - robot.rightFront.getCurrentPosition();

            pDrivingLeft.err = errorLeft;
            pDrivingRight.err = errorRight;

            telemetry.addData("rightPosition", positionRightAverage);
            telemetry.addData("Target", targetInches * RobotMap.TICKS_PER_INCH);
            telemetry.addData("Front Left Motor Position", robot.leftFront.getCurrentPosition());
            telemetry.addData("Front Right Motor Position", robot.rightFront.getCurrentPosition());
            telemetry.addData("Back Left Motor Position", robot.leftBack.getCurrentPosition());
            telemetry.addData("Back Right Motor Position", robot.rightBack.getCurrentPosition());
            telemetry.update();

            double left = pDrivingLeft.getValue(positionLeftAverage);
            double right = pDrivingRight.getValue(positionRightAverage);

            robot.leftFront.setPower(Range.clip(left, -1,1));
            robot.rightFront.setPower(Range.clip(right, -1,1));
        }

        robot.stop();

        ElapsedTime A = new ElapsedTime();
        A.startTime();
        A.reset();
        while(A.seconds()<2){

        }

        // *****************************************************************************************
        // Driving Distance
        // *****************************************************************************************

        //setting target value
        double targetTicks = 17.5 * RobotMap.TICKS_PER_INCH;

        pDrivingLeft.setTarget(robot.leftFront.getCurrentPosition() +targetTicks);
        pDrivingRight.setTarget(robot.rightFront.getCurrentPosition() + targetTicks);

        errorLeft = targetTicks - robot.leftFront.getCurrentPosition();
        errorRight = targetTicks- robot.rightFront.getCurrentPosition();

        //Proportional Loop

        while(opModeIsActive() && errorLeft > RobotMap.DRIVE_TOLERANCE && errorRight > RobotMap.DRIVE_TOLERANCE){
            positionLeftAverage = robot.leftFront.getCurrentPosition();
            positionRightAverage = robot.rightFront.getCurrentPosition();

            errorLeft = targetTicks - robot.leftFront.getCurrentPosition();
            errorRight = targetTicks * RobotMap.TICKS_PER_INCH - robot.rightFront.getCurrentPosition();

            pDrivingLeft.err = errorLeft;
            pDrivingRight.err = errorRight;

            telemetry.addData("rightPosition", positionRightAverage);
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Front Left Motor Position", robot.leftFront.getCurrentPosition());
            telemetry.addData("Front Right Motor Position", robot.rightFront.getCurrentPosition());
            telemetry.addData("Back Left Motor Position", robot.leftBack.getCurrentPosition());
            telemetry.addData("Back Right Motor Position", robot.rightBack.getCurrentPosition());
            telemetry.update();

            double left = pDrivingLeft.getValue(positionLeftAverage);
            double right = pDrivingRight.getValue(positionRightAverage);

            robot.leftFront.setPower(Range.clip(left, -1,1));
            robot.rightFront.setPower(Range.clip(right, -1,1));
        }

        robot.stop();

        A = new ElapsedTime();
        A.startTime();
        A.reset();
        while(A.seconds()<2){

        }


        // *****************************************************************************************
        // Turning
        // *****************************************************************************************
        initR();
        Gyro gyro = new Gyro(hardwareMap);
        imu = gyro.imu;

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        PID pid = new PID(RobotMap.P_TURN);
        double target = gyro.getYaw() - 90 +25;//heading

        pid.setTarget(target);

        pid.getValue(gyro.getYaw());

        ElapsedTime b = new ElapsedTime();
        b.startTime();

        // Loop and update the dashboard
        while (opModeIsActive() && Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE && b.seconds() < 3) {
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err, telemetry);
            Logging.log("target", target, telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE, telemetry);

            double pLeft = -pid.getValueP(gyro.getYaw());
            double pRight = pid.getValueP(gyro.getYaw());

            Logging.log("pLeft", pLeft, telemetry);
            Logging.log("pRight", pLeft, telemetry);
            telemetry.update();

            robot.leftFront.setPower(pLeft);
            robot.rightFront.setPower(pRight);
            robot.leftBack.setPower(pLeft);
            robot.rightBack.setPower(pRight);
        }


        //TODO figure out which orientation

        ElapsedTime m = new ElapsedTime();
        m.startTime();

        robot.intakeRight.setPower(1);
        robot.intakeLeft.setPower(-1);

        while(opModeIsActive() && m.seconds() < 0.20){
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err,telemetry);
            Logging.log("target", target,telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE+8, telemetry);
            telemetry.update();
            robot.flipper.setPosition(1);

            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftBack.setPower(-1);
            robot.rightBack.setPower(-1);
        }

        robot.stop();

        m.reset();

        while(opModeIsActive() && m.seconds() < 0.25){
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err,telemetry);
            Logging.log("target", target,telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE, telemetry);
            telemetry.update();
        }


        // *****************************************************************************************
        // Turning Back 180 degrees
        // *****************************************************************************************

        gyro = new Gyro(hardwareMap);
        imu = gyro.imu;

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        pid = new PID(RobotMap.P_TURN);
        target = gyro.getYaw() - 180;//heading

        pid.setTarget(target);

        pid.getValue(gyro.getYaw());

        ElapsedTime t = new ElapsedTime();
        t.startTime();

        // Loop and update the dashboard
        while (opModeIsActive() && Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE && t.seconds() < 3) {
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err, telemetry);
            Logging.log("target", target, telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE, telemetry);

            double pLeft = -pid.getValueP(gyro.getYaw());
            double pRight = +pid.getValueP(gyro.getYaw());

            Logging.log("pLeft", pLeft, telemetry);
            Logging.log("pRight", pLeft, telemetry);
            telemetry.update();

            robot.leftFront.setPower(pLeft);
            robot.rightFront.setPower(pRight);
            robot.leftBack.setPower(pLeft);
            robot.rightBack.setPower(pRight);
        }


        robot.stop();

        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "multi");
        sensorDistance.getDistance(DistanceUnit.INCH);

        double initialPosLeft = robot.leftFront.getCurrentPosition();
        double initialPosRight = robot.rightFront.getCurrentPosition();

        robot.intakeRight.setPower(1);
        robot.intakeLeft.setPower(-.1);

        ElapsedTime time1 = new ElapsedTime();
        time1.startTime();

        while(opModeIsActive() && !(sensorDistance.getDistance(DistanceUnit.INCH) < 8) && time1.seconds() < 3.25){
            robot.setDrivePower(-.4);
        }

        double timeForward = time1.seconds() + 2;
        time1.reset();

        while(opModeIsActive() && time1.seconds() < timeForward){
            robot.setDrivePower(+.4);
        }

        time1.reset();
        while(opModeIsActive() && time1.seconds() < 0.1){
            robot.setDrivePower(-1);
        }

        robot.stop();
        robot.flipper.setPosition(0);

    }
}
