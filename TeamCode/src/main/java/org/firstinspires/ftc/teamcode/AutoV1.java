package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;

@Autonomous(name = "AutoV1")
public class AutoV1 extends MM_LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        openCV.THRESHOLD = 25;
        telemetry.addData("openCV", openCVStartup);
        telemetry.addData("imu object", robot.imu);
        telemetry.update();
        waitForStart();

        Mat colorImg = openCV.getFrames();
        Mat contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
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
            robot.vectorDrive(0.25, 180);
            Point center = MM_OpenCV.findCenterOfLargest(contours);
            while (center.x < 375 && opModeIsActive()){
                colorImg = openCV.getFrames();
                contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
                contours = MM_OpenCV.findContours(contourable);
                croppedColor = openCV.CropMat(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                center = MM_OpenCV.findCenterOfLargest(contours);
            }
        }
        robot.stop_motors();
        sleep(1000);
        robot.motor_powers(.3);
        sleep(1100);
        robot.stop_motors();
        sleep(2000);

        robot.rotate(0.2, MecanumYellow.CLOCKWISE);
        double rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation > -90 && opModeIsActive()){
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }
        robot.stop_motors();
        sleep(500);

        robot.rotate(0.12, MecanumYellow.COUNTERCLOCKWISE);
        rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation < -90 && opModeIsActive()){
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }

        robot.motor_powers(.35);
        sleep(1000);
        double seenRed = robot.getColor()[0];
        robot.motor_powers(.2);
        while (seenRed < .15 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.stop_motors();
        sleep(500);
        robot.motor_powers(-.2);
        sleep(250);
        robot.stop_motors();
    }
}
