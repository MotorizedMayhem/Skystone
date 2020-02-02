package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Auto BLUE 2", group = "Depot")
public class AutoV2_Blue extends MM_LinearOpMode {
int forward_addition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        robot.FServo.setPosition(.6);
        openCV.THRESHOLD = 25;
        telemetry.update();
        waitForStart();

        int ExtendPosit = -2450;
        int LiftPosit = -575; //525
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(.75);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);



        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

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
                blockArrangement = MM_OpenCV.LEFT;
                forward_addition = 0;
            } else if (scaledXCenter < 0.66) //valid bc we already checked the first
            {
                blockArrangement = MM_OpenCV.CENTER;
                forward_addition =300;
            } else if (scaledXCenter >= 0.66) { //dont use else in case something rlly messed up
                blockArrangement = MM_OpenCV.RIGHT;
                forward_addition =600;
            }

        }
        telemetry.addData("arrangement", blockArrangement);
        telemetry.update();


        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(500,.3);

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
        encoderStrafeRight(150,.2);
        robot.stopMotors();

        //#### SQUARE UP ####
        squareUp(0,.15,1);

        //#### FINISH FAST FORWARD ####
        encoderForward(650,0.25);

        //#### APPROACH BLOCK SLOW ####
        encoderForward(300,.15); //TODO WAS 350
        robot.stopMotors();

        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        LiftPosit = -320;
        ExtendPosit = -2050;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);


        //### BACK AWAY AFTER GRAB ####
        encoderForward(750,-0.35);
        robot.stopMotors();

        //#### ROTATE TO LINE ####
        squareUp(90,.3, 3);
        squareUp(90,.15, 1);

        //#### FORWARD TOWARD RED LINE ####
        encoderForward(2500 + forward_addition,.45);
        robot.stopMotors();


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
        encoderForward(100,-0.40);
        robot.motorPowers(0);

        LiftPosit = -50;
        ExtendPosit = -200;

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);

        sleep(250);
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        //#### SQUARE UP ####
        squareUp(90,.15, 1);

        sleep(750); //give lift time to come down

        //#### BACK OVER LINE ####
        encoderForward(2650 + forward_addition,-0.65);

        //#### SQUARE UP BACK TO STRAIGHT ####
        squareUp(0,.3, 3);
        squareUp(0,.15, 1);

        //#### BACK UP INTO WALL ####
        encoderForward(1200,-.4);
        robot.stopMotors();

        //### FORWARD BEFORE VIDEO DETECT
        encoderForward(500,0.35);
        squareUp(0,.15,1);

        //#### STRAFE TO DETECT BLOCK ####
        robot.vectorDrive(.25,MecanumYellow.RIGHT);
        Point center = new Point (375,0); //starts us with a loop
        int targetPixel = 125;     //LEFT really means right
        boolean right_case =false;
        if (blockArrangement == MM_OpenCV.LEFT){targetPixel = 150; right_case = true;} //TODO 150 is too little, check 300
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
        encoderStrafeRight(135,.2);
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


        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### FINISH FAST FORWARD ####
        encoderForward(650,0.30);


        //#### APPROACH BLOCK SLOW ####
        encoderForward(400,0.2); //TODO was 450
        robot.motorPowers(0);

        //#### IF RIGHT, PIVOT TO THE RIGHT ####
        if (right_case){squareUp(-30,.15);}

        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.5);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        LiftPosit = -320;
        ExtendPosit = -2050;

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);
        sleep(500);


        //#### BACK AWAY AFTER GRAB ####
        encoderForward(1700,-0.30);
        robot.motorPowers(0);

        //#### ROTATE TO LINE ####
        squareUp(90,.3, 3);
        squareUp(90,.15, 1);

        //#### FORWARD TOWARD RED LINE ####
        if (!right_case) {
            encoderForward(3800 + forward_addition, .45);
        }
        else{
            encoderForward(3500, .45);
        }
        robot.stopMotors();


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
        encoderForward(50,-0.40); //TODO was 100
        robot.motorPowers(0);

        LiftPosit = -50;
        ExtendPosit = -200;

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);

        sleep(250);
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        sleep(750); //give block time to come down

        //#### BACK AWAY AFTER DROP INTO THE LINE ####
        encoderForward(300,-0.40);
        robot.motorPowers(0);

        end(); //TODO

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
    }
    private void squareUp(double angle, double power, double range){
        double diff = robot.getIMU_Heading(AngleUnit.DEGREES) - angle;
        if (diff > 0){
            robot.rotate(power, robot.COUNTERCLOCKWISE);
        }
        else if (diff < 0){
            robot.rotate(power, robot.CLOCKWISE);
        }
        while(Math.abs(diff) > range && opModeIsActive()){
            diff = robot.getIMU_Heading(AngleUnit.DEGREES) - angle;
            telemetry.addData("IMU", robot.getIMU_Heading(AngleUnit.DEGREES));
            telemetry.addData("Diff", diff);
            telemetry.update();
        }
        robot.stopMotors();
    }
}
