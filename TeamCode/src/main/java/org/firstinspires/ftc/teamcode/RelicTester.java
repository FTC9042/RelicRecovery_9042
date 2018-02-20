package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Relic Tester", group = "Sensor Test")
public class RelicTester extends OpMode {
    DcMotor relic;
    Servo arm, claw;

    public void init(){
        relic = hardwareMap.dcMotor.get("relic");
        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");

        arm.scaleRange(0,1);
    }

    public void loop(){
        claw.setPosition(1-gamepad2.right_trigger);
        arm.setPosition(0.33);

        if(gamepad2.dpad_up)
            relic.setPower(1);
        else if(gamepad2.dpad_down)
            relic.setPower(-1);
        else
            relic.setPower(0);


        Logging.log("Distance Slider", relic.getCurrentPosition(), telemetry);
        Logging.log("Wrist Position", arm.getPosition(), telemetry);
        Logging.log("Claw Position", claw.getPosition(), telemetry);
        telemetry.update();
    }
}
