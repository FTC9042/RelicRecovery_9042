package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Relic Tester", group = "Sensor Test")
public class RelicTester extends OpMode {
    DcMotor relic;
    Servo arm, claw;

    public void init(){
        relic = hardwareMap.dcMotor.get("relic");
        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");
    }

    public void loop(){
        arm.setPosition(1-gamepad2.right_trigger);
        claw.setPosition(gamepad2.left_trigger);

        if(gamepad2.dpad_up)
            relic.setPower(1);
        else if(gamepad2.dpad_down)
            relic.setPower(-1);
        else
            relic.setPower(0);
    }
}
