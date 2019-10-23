package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

@TeleOp(name="MatTest", group="Computer Vision")
@Disabled
public class MatTest extends LinearOpMode {
    boolean useless = OpenCVLoader.initDebug();
    @Override
    public void runOpMode() {
        // run until the end of the match (driver presses STOP)
        telemetry.addData("startSuccess",useless);
        telemetry.update();

        waitForStart();

        telemetry.addData("trying", "assignment");
        telemetry.update();
        sleep(2000);
        Mat mat = new Mat(5, 5, CvType.CV_8UC4);
        //Mat mat;
        telemetry.addData("trying", "assignment");
        telemetry.update();
        sleep(2000);
        //mat = new Mat();
        telemetry.addData("trying", "success");
        telemetry.update();
        sleep(2000);

        while(opModeIsActive()){ idle(); }
    }
}
