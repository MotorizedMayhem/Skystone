package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

abstract class MM_AutoOpMode extends LinearOpMode {
    MecanumChassis2019 robot = new MecanumChassis2019();
    MM_Vuforia vuforia = new MM_Vuforia();
    MM_OpenCV openCV = new MM_OpenCV();
    boolean openCVStartup;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        vuforia.init(hardwareMap, MM_Vuforia.SHOW_CAMERA.NO_USE_SCREEN);
        openCVStartup = openCV.init(hardwareMap,vuforia.vuforia);
    }

}
