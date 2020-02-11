package org.firstinspires.ftc.teamcode.MM_Classes;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.view.View;
import android.widget.ImageView;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class MM_OpenCV {
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    private FtcRobotControllerActivity activity = null;// = (FtcRobotControllerActivity) hardwareMap.appContext;
    private boolean USE_WEBCAM;
    public int THRESHOLD = 25;

    public MM_OpenCV(boolean USE_WEBCAM){
        this.USE_WEBCAM = USE_WEBCAM;
    }
    public MM_OpenCV(){
        this(MM_Vuforia.USE_PHONECAM); //use phone by default
    }

    final public static int
        NONE = -1,
        LEFT = 3,
        CENTER = 2,
        RIGHT = 1;



    public boolean init(HardwareMap hardwareMap) //temp until we decide if we want a webcam
    {
        activity = (FtcRobotControllerActivity) hardwareMap.appContext;
        //int cameraMonitorViewId = activity.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = cameraName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.setFrameQueueCapacity(2); //was 2
        vuforia.enableConvertFrameToBitmap();
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        return OpenCVLoader.initDebug();

    }
    public boolean init(HardwareMap hardwareMap,VuforiaLocalizer vuforia){
        activity = (FtcRobotControllerActivity) hardwareMap.appContext;
        this.vuforia = vuforia;

        vuforia.setFrameQueueCapacity(2); //was 2
        vuforia.enableConvertFrameToBitmap();
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        return OpenCVLoader.initDebug();
    }

    public Mat getFrames(){ //think about imputing opmode as using this
        /*
        returns Mat FullColorImage
         */
        Image rgb = null;
        VuforiaLocalizer.CloseableFrame frame;
        try { //get frames
            frame = vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e){
            return null;
            //frame = null;
        }
        //check until 2 are returned
        while (frame==null || frame.getNumImages() == 1){ //HIGH CAPACITY TO GET STUCK HERE
            try {
                frame = vuforia.getFrameQueue().take();
            }
            catch (InterruptedException e){
                frame = null;
            }
        }

        //get the image that is rgb
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        if (rgb == null){return null;} //shouldn't happen

        //copy from frame to bitmap
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        //bitmap to mat
        Mat img = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, img);
        return img;

    }
    public static Mat CropMatRed(Mat in, int offTop, int offRight){
        Rect roi = new Rect(new Point(0,offTop), new Point(in.width() - offRight,in.height()));
        return in.submat(roi).clone();
    }
    public static Mat CropMatBlue(Mat in, int offTop, int offLeft){
        Rect roi = new Rect(new Point(offLeft,offTop), new Point(in.width(),in.height()));
        return in.submat(roi).clone();
    }
    public Mat CropMatRed(Mat in){
        if (USE_WEBCAM){
            return CropMatWebcamRed(in);
        }
        else{
            return CropMatPhone(in);
        }
    }
    public Mat CropMatBlue(Mat in){
        return CropMatWebcamBlue(in);
    }

    public static Mat CropMatPhone(Mat in){
        return CropMatRed(in, 300, 400);
    }
    public static Mat CropMatWebcamRed(Mat in){
        return CropMatRed(in, 240, 360); //was 175,300
    }
    public static Mat CropMatWebcamBlue(Mat in){
        return CropMatBlue(in, 240, 385); //was 175,300
    }

    public static Mat Morphology(Mat img){
        Mat output = new Mat(img.size(), img.type()); //creates output like input
        int operation = Imgproc.MORPH_CLOSE; // dilation followed by erosion
        int kernelShape = Imgproc.CV_SHAPE_RECT;
        org.opencv.core.Size kernelSize = new org.opencv.core.Size(7,7); // will require tuning
        Mat KERNEL = Imgproc.getStructuringElement(kernelShape, kernelSize);
        org.opencv.core.Point originPt = new org.opencv.core.Point(-1,-1); //gives middle of the image
        Imgproc.morphologyEx(img, output,operation,KERNEL, originPt , 1);
        return output;
    }

    public static Mat Threshold(Mat img, double thresh){
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2GRAY); //may be unneeded
        Mat output = new Mat(img.size(), img.type()); //creates output like input
        int threshType = Imgproc.THRESH_BINARY_INV;
        double maxVal = 255;
        Imgproc.threshold(img, output, thresh, maxVal, threshType);
        return output;
    }

    public Mat Threshold(Mat img){return Threshold(img, THRESHOLD);}

    public static Mat ProcessImg(Mat colorImg, int threshold, int offTop, int offRight){
        Mat toBeProcessed = colorImg.clone();
        return Morphology(Threshold(CropMatRed(toBeProcessed, offTop, offRight), threshold));
    }
    public static Mat ProcessImgBlue(Mat colorImg, int threshold, int offTop, int offLeft){
        Mat toBeProcessed = colorImg.clone();
        return Morphology(Threshold(CropMatBlue(toBeProcessed, offTop, offLeft), threshold));
    }

    public Mat ProcessImg(Mat colorImg, int threshold){
        Mat toBeProcessed = colorImg.clone();
        if (USE_WEBCAM){return Morphology(Threshold(CropMatWebcamRed(toBeProcessed), threshold));}
        else {return Morphology(Threshold(CropMatPhone(toBeProcessed), threshold));}
    }
    public Mat ProcessImgBlue(Mat colorImg, int threshold){
        Mat toBeProcessed = colorImg.clone();
        if (USE_WEBCAM){return Morphology(Threshold(CropMatWebcamBlue(toBeProcessed), threshold));}
        else {return Morphology(Threshold(CropMatPhone(toBeProcessed), threshold));}
    }


    public static List<MatOfPoint> findContours(Mat img){
        List<MatOfPoint> output = new ArrayList<>();
        Mat hierarchy = new Mat();
        int mode = Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        org.opencv.core.Point offset = new org.opencv.core.Point(-1,-1);
        Imgproc.findContours(img, output,hierarchy, mode, method,offset);
        return output;
    }

    public static int findLargestContourIndex(List<MatOfPoint> contours){
        double[] areas = new double[contours.size()];
        for (int i = 0; i<contours.size(); i++) {
            areas[i] = Imgproc.contourArea(contours.get(i));
        }
        double max = areas[0];
        int max_index = 0;
        for (int i = 0; i < areas.length; i++)
        {
            if (max < areas[i])
            {
                max = areas[i];
                max_index = i;
            }
        }
        return max_index;
    }

    public static Point findCenterOfLargest(List<MatOfPoint> contours, int max_index){
        RotatedRect rect = null;
        try {
            rect = Imgproc.minAreaRect(new MatOfPoint2f((contours.get(max_index)).toArray()));
        }
        catch (Exception e){
            return null;
        }
        return rect.center;
    }

    public static Point findCenterOfLargest(List<MatOfPoint> contours){
        return findCenterOfLargest(contours, findLargestContourIndex(contours));
    }



    public static Mat DISPLAY(Mat colorImg, List<MatOfPoint> contours){
        if (contours.size() == 0){
            return colorImg;
        }
        Imgproc.drawContours(colorImg, contours,-1, new Scalar(140, 235, 52), -1); //bright green
        int max_index = findLargestContourIndex(contours);
        MatOfPoint[] largestArray = new MatOfPoint[]{contours.get(max_index)};
        List<MatOfPoint> largestList = Arrays.asList(largestArray);
        Imgproc.drawContours(colorImg, largestList, -1, new Scalar(0,255,255), -1); //bright blue

        Point center = findCenterOfLargest(contours, max_index);
        if (center==null){center = new Point(0,0);}
        //telemetry.addData("center", rect.center.toString());

        Imgproc.circle(colorImg,center,10,new Scalar(255,0,0),-1); //draw center of rect


        return colorImg;
    }
    public static void printToDisplay(Mat colorImg, HardwareMap hardwareMap){
        final FtcRobotControllerActivity act = (FtcRobotControllerActivity) hardwareMap.appContext;
        Bitmap bmp = null;
        try {
            //Imgproc.cvtColor(printImg, tmp, Imgproc.COLOR_GRAY2RGBA, 4);

            bmp = Bitmap.createBitmap(colorImg.cols(), colorImg.rows(), Bitmap.Config.RGB_565); //RGB_565
            Utils.matToBitmap(colorImg, bmp);
        }
        catch (CvException e){ } //TODO ADD LOG


        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        final Bitmap bmpFinal =  Bitmap.createBitmap(bmp, 0, 0, bmp.getWidth(), bmp.getHeight(), matrix, true);


        //final Bitmap bmpFinal = bmp;


        act.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ImageView bitmapHolder = act.getBitmapDisplay();
                //bitmapHolder.setVisibility(View.VISIBLE);
                bitmapHolder.setImageBitmap(bmpFinal);
                bitmapHolder.setVisibility(View.VISIBLE);
            }
        });
    }
}
