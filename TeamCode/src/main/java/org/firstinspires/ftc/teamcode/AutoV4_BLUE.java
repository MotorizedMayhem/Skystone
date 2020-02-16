package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "Auto BLUE V$", group = "Depot")
public class AutoV4_BLUE extends MM_LinearOpMode {
    int forward_addition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        robot.FServo.setPosition(.85);
        openCV.THRESHOLD = 25;
        telemetry.update();
        waitForStart();



        Pair<Point,Mat> center_final = opencvIdentifyCenter();
        Point blockCenter =  center_final.first;
        Mat finalPrint = center_final.second;
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);

        double scaledXCenter = blockCenter.x / finalPrint.width();
        int blockArrangement = MM_OpenCV.NONE;
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
            forward_addition = 600;
        }

        telemetry.addData("arrangement", blockArrangement);
        telemetry.update();



        //#### EXTEND ARM ####
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
        while(opModeIsActive()){
            idle();
        }

        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(500,.3);

        //#### DETECT BLOCK WHILE STRAFING
        int target = 390;
        if (blockArrangement == MM_OpenCV.LEFT){
            target = 375;
        }
        robot.vectorDrive(0.25, MecanumYellow.LEFT);
        Point center = opencvIdentifyCenter().first;
        while (center.x < target && opModeIsActive()){
            center_final = opencvIdentifyCenter();
            center = opencvIdentifyCenter().first;
            MM_OpenCV.printToDisplay(center_final.second, hardwareMap);
        }


        //#### ALIGN TO BLOCK
        encoderStrafeRight(150,.2);
        robot.stopMotors();

        //#### SQUARE UP ####
        squareUp(0,.15,1);

        //#### FINISH FAST FORWARD ####
        encoderForward(650,0.25);

        //#### APPROACH BLOCK SLOW ####
        encoderForward(300,.15);
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
        encoderForward(150,-0.35);
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

        //TODO DETERMINE IF NEEDED
        //#### BACK UP INTO WALL ####
        encoderForward(1900,-.4); //was 900 and halfway
        robot.stopMotors();

        //### FORWARD BEFORE VIDEO DETECT
        encoderForward(500,0.35);
        squareUp(0,.15,1);

        //TODO CONSIDER MAKING THIS HALF BASED ON MEMORY AND VUFORIA
        //#### STRAFE TO DETECT BLOCK ####
        robot.vectorDrive(.25, MecanumYellow.RIGHT); //was 180
        center = new Point (0,0); //starts us with a loop
        int targetPixel = 125;
        boolean right_case = false;
        if (blockArrangement == MM_OpenCV.RIGHT){targetPixel = 150; right_case = true;}
        while (center.x > targetPixel && opModeIsActive()){
            Mat colorImg = openCV.getFrames();
            Mat contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
            List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
            Mat croppedColor = openCV.CropMatBlue(colorImg);
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
        encoderForward(400,0.2);
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
        encoderForward(200,-0.30);
        robot.motorPowers(0);

        //#### ROTATE TO LINE ####
        squareUp(90,.3, 3);
        squareUp(90,.10, .5);

        //#### FORWARD TOWARD RED LINE ####
        if (!right_case) {
            encoderForward(3800 + forward_addition, .45);
        }
        else{
            encoderForward(3900, .45);
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
        encoderForward(50,-0.40);
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
        encoderForward(500,-0.40);
        robot.motorPowers(0);

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
    private Pair<Point,Mat> opencvIdentifyCenter(){
        Mat colorImg = openCV.getFrames();
        Mat contourable = openCV.ProcessImg(colorImg, openCV.THRESHOLD);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        Mat croppedColor = openCV.CropMatRed(colorImg);
        Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
        Point blockCenter = new Point(0,0);
        if (contours.size() != 0) {
            blockCenter = MM_OpenCV.findCenterOfLargest(contours);

            double scaledXCenter = blockCenter.x / finalPrint.width();
        }
        return new Pair<Point,Mat>(blockCenter, finalPrint);
    }
}
