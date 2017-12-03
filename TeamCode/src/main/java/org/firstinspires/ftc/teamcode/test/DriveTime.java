package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Robot;

@Autonomous(name = "Driving Time", group = "Test")
public class DriveTime extends LinearOpMode{

    public void runOpMode(){
        Robot robot = new Robot(this.hardwareMap);
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stop();

        waitForStart();
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        time.reset();
        while(time.milliseconds() < 4000){
            robot.setDrivePower(0.6f);
        }

        while(time.milliseconds() < 10){
            robot.setDrivePower(-0.6f);
        }

        robot.stop();

        robot.activeOpmode = false;
        robot.stop();
    }
}
