package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_Vuforia;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "AutoV1")
public class AutoV1 extends MM_LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        openCV.THRESHOLD = 25;
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
        robot.vectorDrive(.4,0);
        sleep(1500);

        //slower color detect
        robot.vectorDrive(.2,0);
        double seenRed = robot.getColor()[0];
        while (seenRed < .15 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.vectorDrive(.4,0);
        sleep(1100);
        robot.stop_motors();
        sleep(2000); //drop off block


        robot.vectorDrive(.65,200);//back off and go left
        sleep(2000);

        //TODO SQUARE UP

        robot.vectorDrive(.25,180);
        Point center = new Point (0,0); //starts us with a loop
        while (center.x < 375 && opModeIsActive()){
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
        robot.stop_motors();
        sleep(200);

        //fast back to red
        robot.vectorDrive(.7,0);
        sleep(1000);

        robot.vectorDrive(.2,0);
        seenRed = robot.getColor()[0];
        while (seenRed < .15 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.vectorDrive(.4,0);
        sleep(1100);
        robot.stop_motors();
        sleep(2000); //drop off block


        end();












        /* ################### Old solution with turning #########################

        robot.rotate(0.2, MecanumYellow.CLOCKWISE);
        double rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation > -90 && opModeIsActive()){
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }
        robot.stop_motors();
        sleep(250);

        robot.rotate(0.12, MecanumYellow.COUNTERCLOCKWISE);
        rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation < -90 && opModeIsActive()){
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }

        //quick forward toward line
        robot.motor_powers(.35);
        sleep(500);

        //slower drive until color detect
        double seenRed = robot.getColor()[0];
        robot.motor_powers(.16);
        while (seenRed < .15 && opModeIsActive()){
            seenRed = robot.getColor()[0];
            telemetry.addData("Current Red:", seenRed);
            telemetry.update();
        }
        robot.stop_motors();
        sleep(200);
        //go past the line

        robot.motor_powers(.25);
        sleep(800);
        robot.stop_motors();
        sleep(1000);

        //strafe away from danger zone
        robot.vectorDrive(.25,0);
        sleep(250);


        //back away from delivered block
        robot.motor_powers(-.3);
        sleep(1000);

        //turn toward wall
        robot.rotate(0.22, MecanumYellow.COUNTERCLOCKWISE);

        rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation < 90 && opModeIsActive()){ //ends when 180 goes postiive
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }


        //correct
        robot.rotate(0.12, MecanumYellow.CLOCKWISE);
        rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
        while (rotation > 90 && opModeIsActive()){
            rotation = robot.getIMU_Heading(AngleUnit.DEGREES);
            telemetry.addData("Current IMU:", rotation);
            telemetry.update();
        }
        robot.stop_motors();


        OpenGLMatrix seenPos = vuforia.scanTargets().first;
        float[] translation = null;
        if (seenPos != null) {
            translation = MM_Vuforia.getXYZ(seenPos, DistanceUnit.INCH);
            telemetry.addData("Position","x: %.2f, y: %.2f, z: %.2f", translation[0],translation[1],translation[2]);
            telemetry.update();
        }
        */

    }
}
