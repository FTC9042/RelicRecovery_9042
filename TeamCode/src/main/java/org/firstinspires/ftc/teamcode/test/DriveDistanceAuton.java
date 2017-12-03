package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.Robot;
import org.firstinspires.ftc.teamcode.utilities.RobotMap;

@Autonomous(name = "Driving Distance test", group = "Test")
public class DriveDistanceAuton extends LinearOpMode{

    public void runOpMode(){
        Robot robot = new Robot(this.hardwareMap);
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stop();

        waitForStart();
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float dist = RobotMap.distanceToTicks(5);

        float power = 1;

        robot.driveDistance(dist, this);
        this.opModeIsActive();
        robot.stop();
    }
}
