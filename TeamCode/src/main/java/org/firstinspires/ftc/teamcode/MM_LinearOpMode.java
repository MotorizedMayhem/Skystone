package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract class MM_LinearOpMode extends LinearOpMode {
    //MecanumChassis2019 robot = new MecanumChassis2019();
    MM_Vuforia vuforia = new MM_Vuforia();
    MM_OpenCV openCV = new MM_OpenCV(MM_Vuforia.USE_WEBCAM);
    boolean openCVStartup;
    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        vuforia.vuforia = vuforia.createLocalizer(hardwareMap,MM_Vuforia.NO_USE_SCREEN, MM_Vuforia.USE_WEBCAM); //avoid long init for target id
        openCVStartup = openCV.init(hardwareMap,vuforia.vuforia);
    }

}
