package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Auto BLUE", group = "Depot")
public class AutoV1_BLUE extends MM_LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        openCV.THRESHOLD = 25;
        telemetry.update();
        waitForStart();

        Mat colorImg = openCV.getFrames();
        Mat contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        Mat croppedColor = openCV.CropMatBlue(colorImg);
        Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
        int blockArrangement = MM_OpenCV.NONE;

        if (contours.size() != 0) {
            Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);

            double scaledXCenter = blockCenter.x / finalPrint.width();

            //sleep(2000);
            if (scaledXCenter < 0.33) {
                blockArrangement = MM_OpenCV.RIGHT; //incorrect but imporant for later calcs
            } else if (scaledXCenter < 0.66) //valid bc we already checked the first
            {
                blockArrangement = MM_OpenCV.CENTER;
            } else if (scaledXCenter >= 0.66) {
                blockArrangement = MM_OpenCV.LEFT;  //incorrect but imporant for later calcs
            }
        }
        telemetry.addData("arrangement", blockArrangement);
        telemetry.update();
        if (blockArrangement != MM_OpenCV.NONE){ //maybe was right
            robot.vectorDrive(0.25, MecanumYellow.RIGHT);
            Point center = MM_OpenCV.findCenterOfLargest(contours);
            while (center.x > 125 && opModeIsActive()){
                colorImg = openCV.getFrames();
                contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
                contours = MM_OpenCV.findContours(contourable);
                croppedColor = openCV.CropMatBlue(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                center = MM_OpenCV.findCenterOfLargest(contours);
            }
        }
        robot.stopMotors();
        sleep(1000);

        //forward toward the blocks
        robot.motorPowers(.3);
        sleep(1100);
        robot.stopMotors();
        sleep(1000);

        //back away after having grabbed one
        robot.motorPowers(-.2);
        sleep(200);
        robot.stopMotors();
        sleep(200);

        //strafe toward line
        robot.vectorDrive(.6,MecanumYellow.LEFT);
        double distance = (blockArrangement/3.0) * 1000 + 450; //little sus
        sleep(Math.round(distance));

        //slower color detect
        robot.vectorDrive(.2,MecanumYellow.LEFT);
        double seenBlue = robot.getColor(robot.colorLeft)[2];
        while (seenBlue < .07 && opModeIsActive()){
            seenBlue = robot.getColor(robot.colorLeft)[2];
            telemetry.addData("Current Blue:", seenBlue);
            telemetry.update();
        }
        robot.vectorDrive(.4,MecanumYellow.LEFT);
        sleep(1250);
        robot.stopMotors();
        sleep(2000); //drop off block


        robot.vectorDrive(.65,330);//back off and go left //TODO
        sleep(2300);

        //TODO SQUARE UP

        robot.vectorDrive(.25,MecanumYellow.RIGHT);
        Point center = new Point (375,0); //starts us with a loop
        int targetPixel = 125;     //LEFT really means right
        if (blockArrangement == MM_OpenCV.LEFT){targetPixel = 150;} //TODO 150 is too little, check 300
        while (center.x > targetPixel && opModeIsActive()){
            colorImg = openCV.getFrames();
            contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
            contours = MM_OpenCV.findContours(contourable);
            croppedColor = openCV.CropMatBlue(colorImg);
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

        //forward toward the blocks
        robot.motorPowers(.3);
        sleep(900);
        robot.stopMotors();
        sleep(1000);

        //back away after having grabbed one
        robot.motorPowers(-.2);
        sleep(500);
        robot.motorPowers(-.4);
        sleep(500);
        robot.stopMotors();
        sleep(200);

        //fast back to blue
        robot.vectorDrive(.7,MecanumYellow.LEFT);
        distance = ((blockArrangement/3.0) * 1000) + 1000; //little sus
        sleep(Math.round(distance));

        robot.vectorDrive(.2,MecanumYellow.LEFT);
        seenBlue = robot.getColor(robot.colorLeft)[2];
        while (seenBlue < .07 && opModeIsActive()){
            seenBlue = robot.getColor(robot.colorLeft)[2];
            telemetry.addData("Current Blue:", seenBlue);
            telemetry.update();
        }
        robot.vectorDrive(.4,MecanumYellow.LEFT);
        sleep(1250);
        robot.stopMotors();
        sleep(1000); //drop off block

        robot.vectorDrive(.4, MecanumYellow.RIGHT);
        sleep(500);
        robot.stopMotors();
        end();


    }
}
