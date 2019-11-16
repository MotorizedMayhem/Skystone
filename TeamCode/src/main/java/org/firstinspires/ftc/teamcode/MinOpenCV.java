package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.MM_OpenCVLoader;
import org.opencv.android.MM_StaticHelper;
import org.opencv.android.OpenCVLoader;

@TeleOp(name = "Minimized OpenCV")
public class MinOpenCV extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    @Override
    public void runOpMode(){

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        boolean inited = MM_OpenCVLoader.initImgprocOnly(); //test test boiiiii TODO

        //boolean inited = OpenCVLoader.initDebug(false); //
        telemetry.addData("vuforia status", vuforia.toString());
        telemetry.addData("openCV status", inited);
        //telemetry.addData("Libs", libs);
        telemetry.update();

        waitForStart();

        String libs = MM_OpenCVLoader.getLibraryList();
        telemetry.addData("Libs: ", libs);
        telemetry.update();
        while (opModeIsActive())
        {
            idle();
        }
    }
}
