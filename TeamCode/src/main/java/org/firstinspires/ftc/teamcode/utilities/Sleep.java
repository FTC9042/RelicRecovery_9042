package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by anikaitsingh on 12/3/17.
 */

public class Sleep {

    public static void pause(double sec, Telemetry telemetry){
        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while(time.seconds()<= sec){
            Logging.log("paused ", sec + " seconds", telemetry);
            telemetry.update();
        }
    }
}
