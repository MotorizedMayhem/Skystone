package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;


@Autonomous(name = "TestAuto")
public class TestAutoOpMode extends LinearOpMode {
    //MecanumChassis2019 robot = new MecanumChassis2019();
    DcMotor leftDrive;
    MM_Vuforia vuforia = new MM_Vuforia();
    MM_OpenCV openCV = new MM_OpenCV();
    boolean openCVStartup;
    @Override
    public void runOpMode(){
        //robot.init(hardwareMap);
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        //vuforia.init(hardwareMap, MM_Vuforia.SHOW_CAMERA.NO_USE_SCREEN);
        //openCVStartup = openCV.init(hardwareMap,vuforia.vuforia);
        openCVStartup = openCV.init(hardwareMap);

        telemetry.addData("OpenCV status", openCVStartup);
        //telemetry.addData("robot motor", robot.blDrive);
        telemetry.update();

        waitForStart();
        Mat colorImg = openCV.getFrames();
        Mat contourable = MM_OpenCV.ProcessImg(colorImg,25);
        List<MatOfPoint> contours = MM_OpenCV.findContours(contourable);
        Mat croppedColor = MM_OpenCV.CropMat(colorImg);
        Mat finalPrint = MM_OpenCV.DISPLAY(croppedColor,contours);
        MM_OpenCV.printToDisplay(finalPrint, hardwareMap);
        Point blockCenter = MM_OpenCV.findCenterOfLargest(contours);
        telemetry.addData("center", blockCenter.toString());
        telemetry.update();
        //sleep(2000);
        Arrangement blockArrangement = Arrangement.NONE;
        int totalWidth = finalPrint.width(); //bc about 400 px are useless
        //telemetry.addData("Picture", "%d width , %d height", finalPrint.width(), finalPrint.height() );
        telemetry.addData("Point", blockCenter);
        telemetry.update();
        //sleep(2000);
        double scaledXCenter = blockCenter.x/totalWidth;
        telemetry.addData("Scaled X", scaledXCenter);
        telemetry.update();
        //sleep(2000);
        if (scaledXCenter < 0.33){
            blockArrangement = Arrangement.LEFT;
        }
        else if (scaledXCenter < 0.66) //valid bc we already checked the first
        {
            blockArrangement = Arrangement.CENTER;
        }
        else if (scaledXCenter >= 0.66){ //dont use else in case something rlly messed up
            blockArrangement = Arrangement.RIGHT;
        }

        telemetry.addData("block arrangement", blockArrangement.name()); //see if name is necessary
        telemetry.update();
        leftDrive.setPower(1);
        sleep(500);
        leftDrive.setPower(0);
        //robot.motor_powers(.5);
        //sleep(1000);
        //robot.stop_motors();
        while (opModeIsActive()){
            idle();
        }
    }
    enum Arrangement{
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }
}
