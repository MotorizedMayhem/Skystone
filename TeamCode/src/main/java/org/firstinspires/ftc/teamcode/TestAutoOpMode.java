package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;


@Autonomous(name = "TestAuto")
public class TestAutoOpMode extends MM_AutoOpMode {
    @Override
    public void runOpMode(){
        telemetry.addData("OpenCV status", openCVStartup);
        Mat colorImg = openCV.getFrames();
        Mat contourable = MM_OpenCV.ProcessImg(colorImg,30);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        Mat finalPrint = MM_OpenCV.DISPLAY(colorImg,contours);
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
        Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);
        telemetry.addData("center", blockCenter.toString());
        Arrangement blockArrangement = Arrangement.NONE;
        int totalWidth = finalPrint.width();
        double scaledXCenter = blockCenter.x/totalWidth;
        if (scaledXCenter < (1/3)){
            blockArrangement = Arrangement.LEFT;
        }
        else if (scaledXCenter < (2/3)) //valid bc we already checked the first
        {
            blockArrangement = Arrangement.CENTER;
        }
        else if (scaledXCenter>= (2/3)){ //dont use else in case something rlly messed up
            blockArrangement = Arrangement.RIGHT;
        }

        telemetry.addData("block arrangement", blockArrangement.name()); //see if name is necessary
        robot.motor_powers(.5);
        sleep(1000);
        robot.stop_motors();
    }
    enum Arrangement{
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }
}
