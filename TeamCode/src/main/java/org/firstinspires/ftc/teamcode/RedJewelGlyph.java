package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Red: Jewel and Glyph", group = "Sensor")
public class RedJewelGlyph extends LinearOpMode {

    ColorSensor sensorColor;
    Robot robot;
    BNO055IMU imu;

    public void initR() {
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

        robot.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
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

            red = hsvValues[0] < 30 || hsvValues[0] > 330;
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
            power = .2;
        } else {
            power = -.2;
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

        while (opModeIsActive()) {
            robot.setDrivePower(.2);
            if (time.time() > 6) {
                break;
            }
            telemetry.update();
        }
        robot.setDrivePower(0);

        Gyro gyro = new Gyro(hardwareMap);
        imu = gyro.imu;
        robot = new Robot(hardwareMap);
        initR();


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        PID pid = new PID(RobotMap.P_TURN);
        double target = gyro.getYaw() + 55 - 15;//heading

        pid.setTarget(target);

        pid.getValue(gyro.getYaw());

        ElapsedTime t = new ElapsedTime();
        t.startTime();

        while (opModeIsActive() && t.seconds() < 1) {
            robot.setDrivePower(-0.2);
        }

        t.reset();

        // Loop and update the dashboard
        while (opModeIsActive() && Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE && t.seconds() < 3) {
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err, telemetry);
            Logging.log("target", target, telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE, telemetry);

            double pLeft = pid.getValueP(gyro.getYaw());
            double pRight = -pid.getValueP(gyro.getYaw());

            Logging.log("pLeft", pLeft, telemetry);
            Logging.log("pRight", pLeft, telemetry);
            telemetry.update();

            robot.setDrivePower(pLeft, pRight);
        }

        t.reset();
        while (opModeIsActive() && t.seconds() < 0.4) {
            robot.setDrivePower(1);
            robot.flipper.setPosition(1);
        }
        robot.setDrivePower(0);

        t.reset();
        while (opModeIsActive() && t.seconds() < 0.3){
            robot.setDrivePower(-0.2);
        }
        robot.setDrivePower(0);

        target = gyro.getYaw() - 65 + 15;//heading

        pid.setTarget(target);

        pid.getValue(gyro.getYaw());

        t.reset();

        // Loop and update the dashboard
        while (opModeIsActive() && Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE && t.seconds() < 3) {
            Logging.log("roll: ", gyro.getRoll(), telemetry);
            Logging.log("pitch: ", gyro.getPitch(), telemetry);
            Logging.log("yaw: ", gyro.getYaw(), telemetry);
            Logging.log("error", pid.err, telemetry);
            Logging.log("target", target, telemetry);
            Logging.log("Turn Condition", Math.abs(pid.err) >= RobotMap.TURN_TOLERANCE, telemetry);

            double pLeft = pid.getValueP(gyro.getYaw());
            double pRight = -pid.getValueP(gyro.getYaw());

            Logging.log("pLeft", pLeft, telemetry);
            Logging.log("pRight", pLeft, telemetry);
            telemetry.update();

            robot.setDrivePower(pLeft, pRight);
        }

        t.reset();

        while(opModeIsActive() && t.seconds() < 0.4){
            robot.setDrivePower(-0.5);
        }

        t.reset();
        while(opModeIsActive() && t.seconds() < 0.4){
            robot.setDrivePower(0.5);
        }

        t.reset();
        while(opModeIsActive() && t.seconds() < 0.4){
            robot.setDrivePower(-0.5);
        }

        t.reset();
        while(opModeIsActive() && t.seconds() < 0.25){
            robot.setDrivePower(0.20);
        }
    }
}