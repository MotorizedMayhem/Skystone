package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

        int ExtendPosit = -2450;
        int LiftPosit = -575; //525
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);


        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(500,.3);

        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);


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
            robot.vectorDrive(0.2, 180);
            Point center = MM_OpenCV.findCenterOfLargest(contours);
            while (center.x < 390 && opModeIsActive()){
                colorImg = openCV.getFrames();
                contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
                contours = MM_OpenCV.findContours(contourable);
                croppedColor = openCV.CropMatRed(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                center = MM_OpenCV.findCenterOfLargest(contours);
            }
        }
        encoderStrafeRight(160,-.2);
        robot.stopMotors();
        /*
        //sleep(250);
        ExtendPosit = -2450;
        LiftPosit = -575; //525
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);


        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(500,.3);

        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        */
        //#### FINISH FAST FORWARD ####
        encoderForward(750,0.25);

        //#### APPROACH BLOCK SLOW ####
        encoderForward(400,.15);
        robot.motorPowers(0);

        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        LiftPosit = -250;
        ExtendPosit = -200;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);


        //### BACK AWAY AFTER GRAB ####
        encoderForward(550,-0.35);
        robot.motorPowers(0);

        //#### SQUARE UP ####
        squareUp(0,.2);

        //#### STRAFE TOWARD RED LINE AND CLEAR ####
        encoderStrafeRight(3000,.45);
//        double distance = (blockArrangement/3.0) * 1000 + 100; //little sus
//        sleep(Math.round(distance));

        //#### STRAFE SLOW COLOR DETECT ####
//        robot.vectorDrive(.2,0);
//        double seenRed = robot.getColor(robot.colorRight)[0];
//        while (seenRed < .07 && opModeIsActive()){
//            seenRed = robot.getColor(robot.colorRight)[0];
//            telemetry.addData("Current Red:", seenRed);
//            telemetry.update();
//        }

        //#### SQUARE UP ####
        squareUp(0,.2);


        //#### RELEASE BLOCK ####
        ExtendPosit = -2650;
        LiftPosit = -950;
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);

        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);
        sleep(250);


        //#### BACK AWAY FROM BLOCK ####
        encoderForward(650,-0.40);
        robot.motorPowers(0);

        LiftPosit = -50;
        ExtendPosit = -200;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);
        sleep(250);

        //#### FAST BACK LEFT DIAGONAL ####
        robot.vectorDrive(.85,205);//back off and go left
        waitEncoderAvg(3500);

        //#### POSSIBLY SQUARE UP ####
        squareUp(0, .2);


        //#### STRAFE TO DETECT BLOCK ####
        robot.vectorDrive(.25,180);
        Point center = new Point (0,0); //starts us with a loop
        int targetPixel = 390;
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
        encoderStrafeRight(160,-.2);
        robot.stopMotors();
        //sleep(250);

        //#### RAISE CLAW TO ATTACK ####
        ExtendPosit = -2450;
        LiftPosit = -555;
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);

        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(400,0.30);

        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### FINISH FAST FORWARD ####
        encoderForward(750,0.30);


        //#### APPROACH BLOCK SLOW ####
        encoderForward(400,0.2);
        robot.motorPowers(0);


        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        LiftPosit = -250;
        ExtendPosit = -200;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);
        sleep(500);


        //#### BACK AWAY AFTER GRAB ####
        encoderForward(1000,-0.30);
        robot.motorPowers(0);

        //#### SQUARE UP ####
        squareUp(0,.2);

        //#### FAST STRAFE TO RED LINE AND CLEAR ####
        encoderStrafeRight(5000,.75);
        robot.motorPowers(0);

        //TODO
        robot.stopMotors();
        while(opModeIsActive()){reportMotors();}

        //#### RELEASE BLOCK ####
        ExtendPosit = -2450;
        LiftPosit = -800;
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);

        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);
        sleep(250);

        //#### BACK AWAY FROM BLOCK ####
        encoderForward(350,-0.40);
        robot.motorPowers(0);

        LiftPosit = -50;
        ExtendPosit = -200;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);
        sleep(250);

        //#### BACK AWAY AFTER DROP ####
        encoderForward(500,-0.40);
        robot.motorPowers(0);

        //#### STRAFE BACK OVER RED LINE ####
        encoderStrafeRight(1500,-.7);

        end();

    }
    private void reportMotors(){
        telemetry.addData("fl",robot.flDrive.getCurrentPosition());
        telemetry.addData("fr",robot.frDrive.getCurrentPosition());
        telemetry.addData("bl",robot.blDrive.getCurrentPosition());
        telemetry.addData("br",robot.brDrive.getCurrentPosition());
        telemetry.update();
    }
    private void encoderForward(int distance, double power){
        robot.resetEncoders();
        robot.motorPowers(power);
        while(robot.encoderAvg() < distance && opModeIsActive()){
            reportMotors();
        }

    }
    private void encoderStrafeRight(int distance, double power){
        robot.resetEncoders();
        robot.motorPowers(power,-power,-power,power);
        while(robot.encoderAvg() < distance && opModeIsActive()){
            reportMotors();
        }
    }
    private void waitEncoderAvg(int distance){
        robot.resetEncoders();
        while(robot.encoderAvg() < distance && opModeIsActive()){
            reportMotors();
        }
    }
    private void squareUp(double angle, double power){
        double diff = robot.getIMU_Heading(AngleUnit.DEGREES) - angle;
        if (diff > 0){
            robot.rotate(power, robot.COUNTERCLOCKWISE);
        }
        else if (diff < 0){
            robot.rotate(power, robot.CLOCKWISE);
        }
        while(Math.abs(diff) > 1.5){
            diff = robot.getIMU_Heading(AngleUnit.DEGREES) - angle;
        }
        robot.stopMotors();
        return;
    }
}
