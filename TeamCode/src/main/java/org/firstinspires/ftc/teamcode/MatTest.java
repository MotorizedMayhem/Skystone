package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

@TeleOp(name="MatTest", group="Computer Vision")
public class MatTest extends LinearOpMode {
    Mat mat = new Mat(5, 5, CvType.CV_8UC4);
    @Override
    public void runOpMode() {
        // run until the end of the match (driver presses STOP)
        telemetry.addData("started","started");
        telemetry.update();


        waitForStart();

        telemetry.addData("trying", "declaration");
        telemetry.update();
        sleep(2000);
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
