package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;

@Autonomous(name = "AutoV1")
public class AutoV1 extends MM_LinearOpMode{
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private final int THRESHOLD = 20;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        Mat colorImg = openCV.getFrames();
        Mat contourable = openCV.ProcessImg(colorImg, THRESHOLD);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        //Mat croppedColor = MM_OpenCV.CropMat(colorImg, topOff,rightOff);
        Mat croppedColor = openCV.CropMat(colorImg);
        Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
        MM_OpenCV.Arrangement blockArrangement = MM_OpenCV.Arrangement.NONE;

        if (contours.size() != 0) {
            Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);

            double scaledXCenter = blockCenter.x / finalPrint.width();

            //sleep(2000);
            if (scaledXCenter < 0.33) {
                blockArrangement = MM_OpenCV.Arrangement.LEFT;
            } else if (scaledXCenter < 0.66) //valid bc we already checked the first
            {
                blockArrangement = MM_OpenCV.Arrangement.CENTER;
            } else if (scaledXCenter >= 0.66) { //dont use else in case something rlly messed up
                blockArrangement = MM_OpenCV.Arrangement.RIGHT;
            }
        }
        telemetry.addData("arrangement", blockArrangement.name());
        telemetry.update();
        if (blockArrangement != MM_OpenCV.Arrangement.RIGHT){
            leftDrive.setPower(.12);
            rightDrive.setPower(.12);
            Point center = MM_OpenCV.findCenterOfLargest(contours);
            while (center.x < 375 && opModeIsActive()){
                colorImg = openCV.getFrames();
                contourable = openCV.ProcessImg(colorImg, THRESHOLD);
                contours = MM_OpenCV.findContours(contourable);
                croppedColor = openCV.CropMat(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                center = MM_OpenCV.findCenterOfLargest(contours);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);


    }
}
