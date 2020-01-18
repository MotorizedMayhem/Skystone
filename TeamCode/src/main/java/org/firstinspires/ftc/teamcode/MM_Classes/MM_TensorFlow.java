package org.firstinspires.ftc.teamcode.MM_Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MM_TensorFlow {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public MM_TensorFlow(){}     //empty constructor

    public void init(HardwareMap hardwareMap, double min_confidence, boolean show_camera)
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (!ClassFactory.getInstance().canCreateTFObjectDetector()){
            int killMe = 1/0; //this will throw an exception
        }

        TFObjectDetector.Parameters tfodParameters;
        if (show_camera) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }
        else{
            tfodParameters = new TFObjectDetector.Parameters();
        }

        tfodParameters.minimumConfidence = min_confidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.activate();
    }


    public List<Recognition> findBricks()
    {
        return tfod.getUpdatedRecognitions();
    }

    public Recognition findSkytone() { //looks for skystones, retunrs a non null position if it sees 1 skystone
        List<Recognition> recogs = this.findBricks();
        boolean one_seen = false;
        Recognition skystone = null;
        for (Recognition recog : recogs) {
            if (recog.getLabel() == "Skystone") {
                if (one_seen) {
                    return null;
                } //if this is the second we're seeing, return null
                else {
                    one_seen = true;
                    skystone = recog;
                }
            }
        }
        return skystone; //will be null if we never saw a skystone, or defined if not
    }


    public void stop(){
        tfod.shutdown();
    }

}
