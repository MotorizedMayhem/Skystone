package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MM_OpenCV {
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";


    MM_OpenCV(){

    }

    boolean init() //temp until we decide if we want a webcam
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        return OpenCVLoader.initDebug();

    }

    Mat getFrames(){
        Image rgb = null;
        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e){
            return null;
            //frame = null;
        }
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        if (rgb == null){return null;}
        //TODO FIND A WAY TO CHOP OFF THE TOP HALF
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);
        return tmp;

    }

    static Mat Morphology(Mat img){
        Mat output = new Mat(img.size(), img.type()); //creates output like input
        int operation = Imgproc.MORPH_CLOSE; // dilation followed by erosion
        int kernelShape = Imgproc.CV_SHAPE_RECT;
        org.opencv.core.Size kernelSize = new org.opencv.core.Size(5,5); // will require tuning
        Mat KERNEL = Imgproc.getStructuringElement(kernelShape, kernelSize);
        org.opencv.core.Point originPt = new org.opencv.core.Point(-1,-1); //gives middle of the image
        Imgproc.morphologyEx(img, output,operation,KERNEL, originPt , 1);
        return output;
    }

    static Mat Threshold(Mat img, double thresh){
        Mat output = new Mat(img.size(), img.type()); //creates output like input
        int threshType = Imgproc.THRESH_BINARY_INV;
        double maxVal = 255;
        Imgproc.threshold(img, output, thresh, maxVal, threshType);
        return output;
    }

    static Mat Threshold(Mat img){
        return Threshold(img, 200);
    }

    static List<MatOfPoint> findContours(Mat img){
        List<MatOfPoint> output = new ArrayList<>();
        Mat hierarchy = new Mat();
        int mode = Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        org.opencv.core.Point offset = new org.opencv.core.Point(-1,-1);
        Imgproc.findContours(img, output,hierarchy, mode, method,offset);
        return output;
    }


}
