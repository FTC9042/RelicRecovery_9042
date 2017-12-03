package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by anikaitsingh on 12/2/17.
 */

public class Gyro {
    // The IMU sensor object
    BNO055IMU imu;

    Robot robot;

    Orientation angles;
    Acceleration gravity;
    Telemetry telemetry;

    public Gyro(HardwareMap map, final Telemetry telemetry, LinearOpMode mode) {
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(defaultParam());

        this.telemetry = telemetry;

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public BNO055IMU.Parameters defaultParam() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        return parameters;
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        logGyro();
        return angles.firstAngle;
    }

    public double getPitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        logGyro();
        return angles.secondAngle;
    }

    public double getYaw() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        logGyro();
        return angles.thirdAngle;
    }

    public void logGyro(){
        Logging.log("heading: ", getHeading(), telemetry);
        Logging.log("pitch: ", getPitch(), telemetry);
        Logging.log("yaw: ", getYaw(), telemetry);
//        telemetry.update();
    }

}