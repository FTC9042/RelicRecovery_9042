package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by anikaitsingh on 11/9/17.
 */

@Autonomous(name = "Claw test")
public class TestClaw extends OpMode{
    public Servo left, right;
    public boolean clawOpen = true;

    public void init(){
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        left.setPosition(1);
        right.setPosition(0);
    }

    public void loop(){
        if(gamepad1.a){
            toggleGlyph();
        }
    }

    public void toggleGlyph() {
        if (clawOpen) {
            left.setPosition(1);
            right.setPosition(0);

        } else {
            left.setPosition(0);
            right.setPosition(1);
        }

        clawOpen = !clawOpen;
    }
}
