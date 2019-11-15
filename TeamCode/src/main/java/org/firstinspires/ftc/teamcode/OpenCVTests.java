/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_OpenCV;
import org.opencv.android.MM_OpenCVLoader;
import org.opencv.android.MM_StaticHelper;
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

import java.util.Arrays;
import java.util.List;


@TeleOp(name = "OpenCV Test", group = "Computer Vision")
@Disabled
public class OpenCVTests extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    private WebcamName webcamName;

  @Override
  public void runOpMode(){
      final FtcRobotControllerActivity act = (FtcRobotControllerActivity) hardwareMap.appContext;
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

      //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
      //parameters.cameraName = webcamName;

      //  Instantiate the Vuforia engine
      vuforia = ClassFactory.getInstance().createVuforia(parameters);
      boolean inited = MM_OpenCVLoader.initImgprocOnly(); //test test boiiiii TODO
      String libs = MM_StaticHelper.getLibraryList();
      //boolean inited = MM_StaticHelper.initOpenCV(false); //
      telemetry.addData("vuforia status", vuforia.toString());
      telemetry.addData("openCV status", inited);
      telemetry.addData("Libs", libs);
      telemetry.update();


      waitForStart();
      telemetry.addData("Status", "started");
      telemetry.update();
      vuforia.setFrameQueueCapacity(2); //was 2
      vuforia.enableConvertFrameToBitmap();
      Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);



      Image rgb = null;
      VuforiaLocalizer.CloseableFrame frame;
      try {
          frame = vuforia.getFrameQueue().take();
      }
      catch (InterruptedException e){
          telemetry.addData("exception", "interrupted at getting frame");
          telemetry.update();
          sleep(5000);
          frame = null;
      }
      while (frame.getNumImages() == 1 && opModeIsActive()){
          try {
              frame = vuforia.getFrameQueue().take();
          }
          catch (InterruptedException e){
              telemetry.addData("exception", "interrupted at getting frame");
              telemetry.update();
              sleep(5000);
              frame = null;
          }
      }
      telemetry.addData("frameClass", frame.getClass());
      telemetry.update();

      long numImages = frame.getNumImages();
      telemetry.addData("images", numImages);
      telemetry.update();
      //sleep(500);


      //convertFrameToBitmap

      for (int i = 0; i < numImages; i++) {
          if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
              rgb = frame.getImage(i);
              break;
          }
      }
      telemetry.addData("frameFormat0", frame.getImage(0).getFormat()  ); //grayscale
      //telemetry.addData("frameFormat1", frame.getImage(1).getFormat()  ); //RGB565 //TODO add catch
      telemetry.addData("rgbFormat", rgb.getFormat());
      telemetry.update();
      //sleep(500);
      //TODO FIND A WAY TO CHOP OFF THE TOP HALF

      Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
      bm.copyPixelsFromBuffer(rgb.getPixels());

      telemetry.addData("bitmap", "pixels copied from image"); //RGB565
      telemetry.update();
      //sleep(500);

      // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
      //Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC3);

      Mat img = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC3); //TODO was changed
      telemetry.addData("mat", "created similar mat"); //RGB565
      telemetry.update();
      //sleep(500);
      Utils.bitmapToMat(bm, img);
      telemetry.addData("mat", "bitmap converted to mat"); //RGB565
      telemetry.update();
      //sleep(500);

      //chop off top third
      //Mat newImg = img.submat(img.height()/3, img.height(), img.width(),img.width());
      Rect roi = new Rect(new Point(0,300), new Point(img.width(),img.height()));
      Mat newImg = img.submat(roi);

      Mat colorImg = newImg.clone(); //was img
      telemetry.addData("newImg", newImg);
      telemetry.update();

      // convert to grayscale before returning
      Imgproc.cvtColor(newImg, newImg, Imgproc.COLOR_RGB2GRAY);
      telemetry.addData("mat", "mat converted to grayscale"); //RGB565
      telemetry.update();
      //sleep(500);

      frame.close();









      telemetry.addData("Status", "frames");
      telemetry.addData("img", newImg.toString());
      telemetry.update();
      //sleep(500);
      

      
      Mat threshed = MM_OpenCV.Threshold(newImg, 30).clone(); //was 75
      telemetry.addData("Status", "threshed");
      telemetry.update();
      //sleep(500);

      Mat morphed = MM_OpenCV.Morphology(threshed);
      telemetry.addData("Status", "morphed");
      telemetry.update();
      //sleep(500);

      List<MatOfPoint> contours = MM_OpenCV.findContours(morphed);
      telemetry.addData("Status", "contoured");
      telemetry.addData("contour length", contours.size());
      telemetry.update();
      //sleep(500);


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

      Imgproc.drawContours(colorImg, contours,-1, new Scalar(140, 235, 52), -1); //bright green
      MatOfPoint[] largestArray = new MatOfPoint[]{contours.get(max_index)};
      List<MatOfPoint> largestList = Arrays.asList(largestArray);
      Imgproc.drawContours(colorImg, largestList, -1, new Scalar(0,255,255), -1); //bright blue

      RotatedRect rect = null;
      try {
           rect = Imgproc.minAreaRect(new MatOfPoint2f((contours.get(max_index)).toArray()));
      }
      catch (Exception e){
          telemetry.addData("exception", e);
          telemetry.update();
      }
      //telemetry.addData("center", rect.center.toString());

      Imgproc.circle(colorImg,rect.center,10,new Scalar(255,0,0),-1); //draw center of rect

      Bitmap bmp = null;
      try {
          //Imgproc.cvtColor(printImg, tmp, Imgproc.COLOR_GRAY2RGBA, 4);

          bmp = Bitmap.createBitmap(colorImg.cols(), colorImg.rows(), Bitmap.Config.RGB_565); //RGB_565
          Utils.matToBitmap(colorImg, bmp);
      }

      catch (CvException e){
          telemetry.addData("Exception", e);
          telemetry.update();
      } //TODO ADD LOG


      Matrix matrix = new Matrix();
      matrix.postRotate(90);
      final Bitmap bmpFinal =  Bitmap.createBitmap(bmp, 0, 0, bmp.getWidth(), bmp.getHeight(), matrix, true);


      //final Bitmap bmpFinal = bmp;


      act.runOnUiThread(new Runnable() {
          @Override
          public void run() {
              ImageView bitmapHolder = act.getBitmapDisplay();
              bitmapHolder.setImageBitmap(bmpFinal);
          }
      });

      telemetry.addData("Status", "reached end");
      telemetry.addData("center", rect.center.toString());
      telemetry.update();
      while (opModeIsActive()){

          idle();
      }

  }


}


