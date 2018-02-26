import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by anikaitsingh on 2/24/18.
 */
@Autonomous(name = "Distance Test", group = "Test")
public class DistanceTest extends OpMode{
    DistanceSensor multiGlyph;

    @Override
    public void init() {
        multiGlyph = hardwareMap.get(DistanceSensor.class, "multi");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", multiGlyph.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
