package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by anikaitsingh on 2/5/18.
 */

@Autonomous(name = "Tick Counter", group = "Senosr")
public class TickCounter extends OpMode{

    public DcMotor leftFront, leftBack, rightFront, rightBack;

    public void init(){
        HardwareMap map = this.hardwareMap;
        leftFront = map.dcMotor.get("1");
        leftBack = map.dcMotor.get("2"); // changed originally rightFront
        rightFront = map.dcMotor.get("3");
        rightBack = map.dcMotor.get("4");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop(){
        Logging.log("Front Left Motor Position", leftFront.getCurrentPosition(),telemetry);
        Logging.log("Front Right Motor Position", rightFront.getCurrentPosition(),telemetry);
        Logging.log("Back Left Motor Position", leftBack.getCurrentPosition(),telemetry);
        Logging.log("Back Right Motor Position", rightBack.getCurrentPosition(),telemetry);

    }
}
