package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;


@TeleOp(name = "IdentifyBlock")
public class OpenCVLoop extends MM_LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        telemetry.addData("OpenCV status", openCVStartup);
        //telemetry.addData("robot motor", robot.blDrive);
        telemetry.update();

        waitForStart();
        int threshold = 20;
        int rightOff = 325;
        int topOff = 175;
        while (opModeIsActive()) {
            Mat colorImg = openCV.getFrames();
            if (gamepad1.dpad_up){
                topOff--;
            }
            if (gamepad1.dpad_down){
                topOff++;
            }
            if (gamepad1.dpad_left){
                rightOff++;
            }
            if (gamepad1.dpad_right){
                rightOff--;
            }
            //telemetry.addData("threshold", threshold);
            telemetry.addData("top", topOff);
            telemetry.addData("right", rightOff);
            //Mat contourable = MM_OpenCV.ProcessImg(colorImg, threshold, topOff, rightOff);
            Mat contourable = openCV.ProcessImg(colorImg, threshold);
            List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
            //Mat croppedColor = MM_OpenCV.CropMat(colorImg, topOff,rightOff);
            Mat croppedColor = openCV.CropMat(colorImg);
            Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
            MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
            Arrangement blockArrangement = Arrangement.NONE;
            if (contours.size() != 0) {
                Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);

                double scaledXCenter = blockCenter.x / finalPrint.width();

                //sleep(2000);
                if (scaledXCenter < 0.33) {
                    blockArrangement = Arrangement.LEFT;
                } else if (scaledXCenter < 0.66) //valid bc we already checked the first
                {
                    blockArrangement = Arrangement.CENTER;
                } else if (scaledXCenter >= 0.66) { //dont use else in case something rlly messed up
                    blockArrangement = Arrangement.RIGHT;
                }
                telemetry.addData("Block Center", "( %.2f , %.2f )", blockCenter.x, blockCenter.y);
            }

            //telemetry.addData("Block Center",blockCenter.toString());
            telemetry.addData("block arrangement", blockArrangement.name()); //see if name is necessary
            telemetry.update();
        }
    }
    enum Arrangement{
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

}
