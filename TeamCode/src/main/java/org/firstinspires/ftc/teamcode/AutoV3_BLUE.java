package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpMode;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_LinearOpModeV2;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;
@Autonomous(name = "Auto Depot BLUE", group = "Depot")
public class AutoV3_BLUE extends MM_LinearOpModeV2 {
private int forward_addition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        robot.FServo.setPosition(.85);
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
                blockArrangement = MM_OpenCV.LEFT;
                forward_addition = 0;
            } else if (scaledXCenter < 0.66) //valid bc we already checked the first
            {
                blockArrangement = MM_OpenCV.CENTER;
                forward_addition =600;
            } else if (scaledXCenter >= 0.66) { //dont use else in case something rlly messed up
                blockArrangement = MM_OpenCV.RIGHT;
                forward_addition = 1200;
            }
        }
        String arrangement = "";
        switch (blockArrangement){
            case (1):
                arrangement = "LEFT";
            case (2):
                arrangement = "CENTER";
            case (3):
                arrangement = "RIGHT";


        }
        telemetry.addData("arrangement", blockArrangement);
        telemetry.addData("position", blockArrangement);
        telemetry.update();

        int ExtendPosit = -2450;
        int LiftPosit = -700; //575
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(.75);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);



        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.65);
        robot.RServo.setPosition(1);


//        //#### FORWARD TOWARD BLOCKS ####
//        encoderForward(500,.3);
//        int target = 125;
//        if (blockArrangement == MM_OpenCV.LEFT){
//            target = 150;
//        }

        //#### FORWARD TOWARD BLOCKS ####
        encoderForward(500,.3);


        robot.vectorDrive(0.25, MecanumYellow.RIGHT);
        Point center = MM_OpenCV.findCenterOfLargest(contours);
        while (center.x > 225 && opModeIsActive()){ //was 125
            colorImg = openCV.getFrames();
            contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
            contours = MM_OpenCV.findContours(contourable);
            croppedColor = openCV.CropMatBlue(colorImg);
            finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
            MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
            center = MM_OpenCV.findCenterOfLargest(contours);
        }

        if (blockArrangement != MM_OpenCV.LEFT){
            //encoderStrafeRight(75,-.2); //was 150
        }
        robot.stopMotors();

        //#### SQUARE UP ####
        squareUp(0,.15,1);

        //#### FINISH FAST FORWARD ####
        encoderForward(650,0.25);

        //#### APPROACH BLOCK SLOW ####
        encoderForward(200,.15); //was 300
        robot.stopMotors();

        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.65);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        moveArmLift(-2050,-320);


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
        moveArmLift(-2650,-950);

        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.65);
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
        squareUp(87,.15, 0.5);

        sleep(750); //give lift time to come down


        //#### BACK OVER LINE ####
        encoderForward(2750 + forward_addition,-0.65);

        //#### SQUARE UP BACK TO STRAIGHT ####
        squareUp(0,.3, 3);
        squareUp(0,.15, 1);

        //#### BACK UP INTO WALL ####
        encoderForward(900,-.4); //fast first
        encoderForward(600,-.3); //slower contact
        if(blockArrangement == MM_OpenCV.RIGHT){
            encoderStrafeRight(500,.4);
        }
        robot.stopMotors();


        //### FORWARD BEFORE VIDEO DETECT
        encoderForward(500,0.35);
        squareUp(0,.15,1);

        //#### STRAFE TO DETECT BLOCK ####
        robot.vectorDrive(.25, MecanumYellow.RIGHT); //was 180
        center = new Point (500,0); //starts us with a loop
        int targetPixel = 225;
        boolean right_case = false;
        if (blockArrangement == MM_OpenCV.RIGHT){right_case = true;}
        else {
            while (center.x > targetPixel && opModeIsActive()) {
                colorImg = openCV.getFrames();
                contourable = openCV.ProcessImgBlue(colorImg, openCV.THRESHOLD);
                contours = MM_OpenCV.findContours(contourable);
                croppedColor = openCV.CropMatBlue(colorImg);
                finalPrint = MM_OpenCV.DISPLAY(croppedColor, contours);
                MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
                if (contours.size() != 0) {
                    int index = MM_OpenCV.findLargestContourIndex(contours);
                    double largestArea = Imgproc.contourArea(contours.get(index));
                    if (largestArea > 5000) {
                        center = MM_OpenCV.findCenterOfLargest(contours, index);
                    }
                }

            }
        }
        //encoderStrafeRight(135,.2);
        robot.stopMotors();
        //sleep(250);

        //#### RAISE CLAW TO ATTACK ####
        moveArmLift(-2450,right_case ? -675: -625); //650, 600


        //#### OPEN CLAW TO ATTACK ####
        robot.LServo.setPosition(1);
        robot.MServo.setPosition(0.65);
        robot.RServo.setPosition(1);

        //#### FINISH FAST FORWARD ####
        encoderForward(650,0.30);


        //#### APPROACH BLOCK SLOW ####
        if (!right_case) {
            encoderForward(300, 0.2);
        }
        robot.motorPowers(0);

        //#### IF RIGHT, PIVOT TO THE RIGHT AND EXTEND MORE ####
        if (right_case){
            squareUp(-25,.15);
            moveArmLift(-3200,600);
        }

        sleep(300);




        //#### GRAB BLOCK #####
        robot.LServo.setPosition(0.3);
        robot.MServo.setPosition(0.65);
        robot.RServo.setPosition(1);

        //#### RETRACT ARM ####
        LiftPosit = -320;
        ExtendPosit = -2050; //was 2050

        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition((int)LiftPosit);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition((int)ExtendPosit);
        sleep(500);


        //#### BACK AWAY AFTER GRAB ####
        encoderForward(right_case ? 50: 200,-0.30); //200
        robot.motorPowers(0);

        //#### ROTATE TO LINE ####
        squareUp(90,.3, 3);
        squareUp(90,.10, .5);


        //#### FORWARD TOWARD RED LINE ####
        if (!right_case) {
            encoderForward(3800 + forward_addition, .45);
        }
        else{
            encoderForward(4250, .45);
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
        robot.MServo.setPosition(0.65);
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
        encoderForward(400,-0.40);
        robot.motorPowers(0);

//        end();

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
        while(Math.abs(diff) > 1.5 && opModeIsActive()){
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
    private void moveArmLift(int extend, int lift){
        extend = -Math.abs(extend);
        lift = -Math.abs(lift);
        robot.lift.setPower(1);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition(lift);

        robot.extend.setPower(1);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setTargetPosition(extend);
    }
}
