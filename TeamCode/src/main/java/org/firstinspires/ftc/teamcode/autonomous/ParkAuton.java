package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Robot;

/**
 * Created by anikaitsingh on 11/1/17.
 */

@Autonomous(name = "Park Auton")
public class ParkAuton extends LinearOpMode{

    public void runOpMode(){
        Robot robot = new Robot(this.hardwareMap);

        waitForStart();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        robot.setDrivePower(.5f);
        while(time.seconds() < 3){
            telemetry.addData("Time", time.seconds());
        }

        robot.setDrivePower(0);
        time.reset();
        while(time.seconds() < 0.3) {
            telemetry.addData("Stopped robot: ", "robot");
        }

        time.reset();
        robot.setDrivePower(-.5f);
        while(time.seconds() < 0.3) {
            telemetry.addData("Time", time.seconds());
        }

        robot.setDrivePower(0);
    }

}
