package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
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
        Mat croppedColor = openCV.CropMatRed(colorImg);
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
                croppedColor = openCV.CropMatRed(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                center = MM_OpenCV.findCenterOfLargest(contours);
            }
        }
        robot.stopMotors();
        sleep(1000);

        //#### FORWARD TOWARD BLOCKS ####
        robot.motorPowers(.5);
        sleep(1100);
        robot.stopMotors();
        sleep(1000);

        //#### APPROACH BLOCK SLOW ####


        //#### GRAB BLOCK #####


        //### BACK AWAY AFTER GRAB ####
        robot.motorPowers(-.2);
        sleep(200);
        robot.stopMotors();
        sleep(200);

        //#### STRAFE TOWARD RED LINE ####
        robot.vectorDrive(.6,0);
        double distance = (blockArrangement/3.0) * 1000 + 100; //little sus
        sleep(Math.round(distance));

        //#### STRAFE SLOW COLOR DETECT ####
        robot.vectorDrive(.2,0);
        double seenRed = robot.getColor(robot.colorRight)[0];
        while (seenRed < .07 && opModeIsActive()){
            seenRed = robot.getColor(robot.colorRight)[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }

        //#### STRAFE OVER AND CLEAR LINE ####
        robot.vectorDrive(.55,0);
        sleep(1250);
        robot.stopMotors();
        sleep(2000); //drop off block

        //#### RELEASE BLOCK ####


        //#### FAST BACK LEFT DIAGONAL ####
        robot.vectorDrive(.8,200);//back off and go left
        sleep(2300);

        //#### POSSIBLY SQUARE UP ####

        //#### STRAFE TO DETECT BLOCK ####
        robot.vectorDrive(.25,180);
        Point center = new Point (0,0); //starts us with a loop
        int targetPixel = 375;
        if (blockArrangement == MM_OpenCV.LEFT){targetPixel = 300;} //TODO 150 is too little, check 300
        while (center.x < targetPixel && opModeIsActive()){
            colorImg = openCV.getFrames();
            contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
            contours = MM_OpenCV.findContours(contourable);
            croppedColor = openCV.CropMatRed(colorImg);
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

        robot.stopMotors();
        sleep(500);

        //#### FORWARD TOWARD BLOCKS ####
        robot.motorPowers(.3);
        sleep(1050);
        robot.stopMotors();
        sleep(1000);

        //#### APPROACH BLOCK SLOW ####


        //#### GRAB BLOCK #####
        

        //#### BACK AWAY AFTER GRAB ####
        robot.motorPowers(-.2);
        sleep(500);
        robot.motorPowers(-.4);
        sleep(500);
        robot.stopMotors();
        sleep(200);

        //#### FAST STRAFE TO RED LINE ####
        robot.vectorDrive(.75,0);
        distance = ((blockArrangement/3.0) * 1500) + 500; //little sus
        sleep(Math.round(distance));

        //#### STRAFE SLOW COLOR DETECT ####
        robot.vectorDrive(.2,0);
        seenRed = robot.getColor(robot.colorRight)[0];
        while (seenRed < .07 && opModeIsActive()){
            seenRed = robot.getColor(robot.colorRight)[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }

        
        
        //#### STRAFE OVER AND CLEAR LINE ####
        robot.vectorDrive(.55,0);
        sleep(1250);
        robot.stopMotors();
        sleep(1000); //drop off block

        //#### RELEASE BLOCK ####
        

        //#### STRAFE BACK OVER RED LINE ####
        robot.vectorDrive(.4, 180);
        sleep(500);
        robot.stopMotors();
        end();


    }
}
