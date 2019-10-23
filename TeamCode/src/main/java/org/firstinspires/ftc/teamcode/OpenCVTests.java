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

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;


@TeleOp(name = "OpenCV Test", group = "Computer Vision")
public class OpenCVTests extends LinearOpMode {
    VuforiaLocalizer vuforia;
    String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
;

  @Override
  public void runOpMode() throws InterruptedException {
      final FtcRobotControllerActivity act = (FtcRobotControllerActivity) hardwareMap.appContext;
      int cameraMonitorViewId = act.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

      //  Instantiate the Vuforia engine
      vuforia = ClassFactory.getInstance().createVuforia(parameters);
      boolean inited = OpenCVLoader.initDebug();
      telemetry.addData("vuforia status", vuforia.toString());
      telemetry.addData("openCV status", inited);
      telemetry.update();


      waitForStart();
      telemetry.addData("Status", "started");
      telemetry.update();
      vuforia.setFrameQueueCapacity(3); //was 2
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
      telemetry.addData("frameClass", frame.getClass());
      telemetry.update();

      long numImages = frame.getNumImages();
      telemetry.addData("images", numImages);
      telemetry.update();
      sleep(500);


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
      sleep(500);
      //TODO FIND A WAY TO CHOP OFF THE TOP HALF

      Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
      bm.copyPixelsFromBuffer(rgb.getPixels());

      telemetry.addData("bitmap", "pixels copied from image"); //RGB565
      telemetry.update();
      sleep(500);

      // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
      //Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC3);

      Mat img = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC3); //TODO was changed
      telemetry.addData("mat", "created similar mat"); //RGB565
      telemetry.update();
      sleep(500);
      Utils.bitmapToMat(bm, img);
      telemetry.addData("mat", "bitmap converted to mat"); //RGB565
      telemetry.update();
      sleep(500);

      Mat colorImg = img.clone();
      // convert to grayscale before returning
      Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2GRAY);
      telemetry.addData("mat", "mat converted to grayscale"); //RGB565
      telemetry.update();
      sleep(500);

      frame.close();









      telemetry.addData("Status", "frames");
      telemetry.addData("img", img.toString());
      telemetry.update();
      sleep(500);
      Mat threshed = MM_OpenCV.Threshold(img, 75).clone();
      telemetry.addData("Status", "threshed");
      telemetry.update();
      sleep(500);
      Mat morphed = MM_OpenCV.Morphology(threshed);
      telemetry.addData("Status", "morphed");
      telemetry.update();
      sleep(500);
      List<MatOfPoint> contours = MM_OpenCV.findContours(morphed);
      telemetry.addData("Status", "contoured");
      telemetry.addData("contour length", contours.size());
      telemetry.update();
      sleep(500);

      Imgproc.drawContours(colorImg, contours,-1, new Scalar(50,100,255), -1);


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
      telemetry.update();
      while (opModeIsActive()){

          idle();
      }

  }


}


