package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_Vuforia;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Auto RED", group = "Depot")
public class AutoV1_RED extends MM_LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        openCV.THRESHOLD = 25;
        telemetry.update();
        waitForStart();

        Mat colorImg = openCV.getFrames();
        Mat contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        Mat croppedColor = openCV.CropMat(colorImg);
        Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
        int blockArrangement = MM_OpenCV.NONE;

        if (contours.size() != 0) {
            Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);

            double scaledXCenter = blockCenter.x / finalPrint.width();

            //sleep(2000);
            if (scaledXCenter < 0.33) {
                blockArrangement = MM_OpenCV.LEFT;
            } else if (scaledXCenter < 0.66) //valid bc we already checked the first
            {
                blockArrangement = MM_OpenCV.CENTER;
            } else if (scaledXCenter >= 0.66) { //dont use else in case something rlly messed up
                blockArrangement = MM_OpenCV.RIGHT;
            }
        }
        telemetry.addData("arrangement", blockArrangement);
        telemetry.update();
        if (blockArrangement != MM_OpenCV.NONE){ //maybe was right
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

        //forward toward the blocks
        robot.motor_powers(.3);
        sleep(1100);
        robot.stop_motors();
        sleep(1000);

        //back away after having grabbed one
        robot.motor_powers(-.2);
        sleep(200);
        robot.stop_motors();
        sleep(200);

        //strafe toward line
        robot.vectorDrive(.6,0);
        double distance = (blockArrangement/3.0) * 1000 + 400; //little sus
        sleep(Math.round(distance));

        //slower color detect
        robot.vectorDrive(.2,0);
        double seenRed = robot.getColor()[0];
        while (seenRed < .10 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.vectorDrive(.4,0);
        sleep(500);
        robot.stop_motors();
        sleep(2000); //drop off block


        robot.vectorDrive(.65,200);//back off and go left
        sleep(2300);

        //TODO SQUARE UP

        robot.vectorDrive(.25,180);
        Point center = new Point (0,0); //starts us with a loop
        int targetPixel = 375;
        if (blockArrangement == MM_OpenCV.LEFT){targetPixel = 300;} //TODO 150 is too little, check 300
        while (center.x < targetPixel && opModeIsActive()){
            colorImg = openCV.getFrames();
            contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
            contours = MM_OpenCV.findContours(contourable);
            croppedColor = openCV.CropMat(colorImg);
            finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
            MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
            if (contours.size() != 0) {
                int index = MM_OpenCV.findLargestContourIndex(contours);
                double largestArea =  Imgproc.contourArea(contours.get(index));
                if (largestArea > 5000){
                    center = MM_OpenCV.findCenterOfLargest(contours,index);
                }
            }

        }

        robot.stop_motors();
        sleep(500);

        //forward toward the blocks
        robot.motor_powers(.3);
        sleep(1050);
        robot.stop_motors();
        sleep(1000);

        //back away after having grabbed one
        robot.motor_powers(-.2);
        sleep(500);
        robot.motor_powers(-.4);
        sleep(500);
        robot.stop_motors();
        sleep(200);

        //fast back to red
        robot.vectorDrive(.7,0);
        distance = ((blockArrangement/3.0) * 1000) + 1000; //little sus
        sleep(Math.round(distance));

        robot.vectorDrive(.2,0);
        seenRed = robot.getColor()[0];
        while (seenRed < .10 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.vectorDrive(.4,0);
        sleep(500);
        robot.stop_motors();
        sleep(1000); //drop off block

        robot.vectorDrive(.4, 180);
        sleep(500);
        robot.stop_motors();
        end();


    }
}
