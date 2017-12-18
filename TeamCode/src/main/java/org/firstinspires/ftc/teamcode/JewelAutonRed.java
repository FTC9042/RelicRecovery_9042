package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
@Autonomous(name = "Red: Jewel Auton", group = "Sensor")
public class JewelAutonRed extends LinearOpMode {

    ColorSensor sensorColor;
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this.hardwareMap);

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

            red = hsvValues[0] < 25 || hsvValues[0] > 330;
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
        if(red){
            power = .2;
        }else{
            power = -.2;
        }

        //for testing purposes
        time.reset();
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            robot.setDrivePower(power);
            if(time.time()>0.5){
                break;
            }
            telemetry.addData("Detected", str);
            telemetry.update();
        }
        robot.jewel.setPosition(1);

        while (opModeIsActive()) {
            robot.setDrivePower(power);
            if(time.time()>5) {
                break;
            }
            telemetry.addData("Detected", str);
            telemetry.update();
        }
        robot.setDrivePower(0);

        if(!red) {
            time.reset();
            Gyro gyro = new Gyro(hardwareMap);
            BNO055IMU imu = gyro.imu;
            robot = new Robot(hardwareMap);
            waitForStart();

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            //TODO figure out why there is a 13 degree error
            double target = imu.getAngularOrientation().firstAngle + 90;//heading

            double error = target- gyro.getYaw();

            // Loop and update the dashboard
            while (opModeIsActive() && Math.abs(error) >= RobotMap.TURN_TOLERANCE) {
                error = target - gyro.getYaw();
                Logging.log("roll: ", gyro.getRoll(), telemetry);
                Logging.log("pitch: ", gyro.getPitch(), telemetry);
                Logging.log("yaw: ", gyro.getYaw(), telemetry);
                Logging.log("error: ", error, telemetry);
                telemetry.update();
                robot.smoothIntake(-1);
                robot.setDrivePower(error*RobotMap.P_TURN, -error*RobotMap.P_TURN);
            }

            robot.setDrivePower(power);
            while (time.seconds() < 0.3 && opModeIsActive()) {
                telemetry.addData("Time", time.seconds());
            }

            robot.setDrivePower(-power);
            while (time.seconds() < 0.3 && opModeIsActive()) {
                telemetry.addData("Time", time.seconds());
            }
        }else{
            time.reset();
            robot.setDrivePower(-power);
            while (time.seconds() < 0.3 && opModeIsActive()) {
                telemetry.addData("Time", time.seconds());
            }
        }

        robot.setDrivePower(0);

        while(opModeIsActive()){
            robot.jewel.setPosition(1);
        }
    }
}
