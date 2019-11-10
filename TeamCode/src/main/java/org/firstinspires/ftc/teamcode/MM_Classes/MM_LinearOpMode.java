package org.firstinspires.ftc.teamcode.MM_Classes;

import android.content.Context;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public abstract class MM_LinearOpMode extends LinearOpMode {
    //MecanumChassis2019 robot = new MecanumChassis2019();
    public MecanumYellow robot = new MecanumYellow();
    public MM_Vuforia vuforia = new MM_Vuforia();
    public MM_OpenCV openCV = new MM_OpenCV(MM_Vuforia.USE_WEBCAM);
    public boolean openCVStartup;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        vuforia.vuforia= vuforia.init(hardwareMap,MM_Vuforia.NO_USE_SCREEN, MM_Vuforia.USE_WEBCAM);
        telemetry.addData("Vuforia", vuforia.vuforia);
        //telemetry.update();
        //vuforia.vuforia = vuforia.createLocalizer(hardwareMap,MM_Vuforia.NO_USE_SCREEN, MM_Vuforia.USE_WEBCAM); //avoid long init for target id
        openCVStartup = openCV.init(hardwareMap,vuforia.vuforia);
        telemetry.addData("OpenCV", openCVStartup);
        telemetry.update();
    }

    public void end(){
        final FtcRobotControllerActivity context = (FtcRobotControllerActivity) hardwareMap.appContext;
        context.runOnUiThread(new Runnable(){
            @Override
            public void run(){
                context.getBitmapDisplay().setVisibility(View.GONE);
            }
        });

    }
}
