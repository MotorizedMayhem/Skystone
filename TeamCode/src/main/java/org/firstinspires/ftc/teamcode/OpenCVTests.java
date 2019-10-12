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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import java.util.List;


@TeleOp(name = "OpenCV Test", group = "Computer Vision")
public class OpenCVTests extends LinearOpMode {
    VuforiaLocalizer vuforia;
    String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

  @Override
  public void runOpMode() throws InterruptedException {
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

      //  Instantiate the Vuforia engine
      vuforia = ClassFactory.getInstance().createVuforia(parameters);
      telemetry.addData("vuforia status", vuforia.toString());
      telemetry.update();

      waitForStart();
      telemetry.addData("Status", "started");
      telemetry.update();
      vuforia.setFrameQueueCapacity(1);
      vuforia.enableConvertFrameToBitmap();
      Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);



      Image rgb = null;
      VuforiaLocalizer.CloseableFrame frame;
      try {
          frame = vuforia.getFrameQueue().take();
      }
      catch (InterruptedException e){
          telemetry.addData("exception", "interrupted");
          telemetry.update();
          sleep(2000);
          frame = null;
      }
      telemetry.addData("frameClass", frame.getClass());
      telemetry.update();

      long numImages = frame.getNumImages();
      telemetry.addData("images", numImages);
      telemetry.update();
      sleep(2000);


      //convertFrameToBitmap

      for (int i = 0; i < numImages; i++) {
          if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
              rgb = frame.getImage(i);
              break;
          }
      }
      telemetry.addData("frameFormat0", frame.getImage(0).getFormat()  ); //grayscale
      telemetry.addData("frameFormat1", frame.getImage(1).getFormat()  ); //RGB565 //TODO add catch
      telemetry.update();
      sleep(5000);
      //TODO FIND A WAY TO CHOP OFF THE TOP HALF

      Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
      bm.copyPixelsFromBuffer(rgb.getPixels());

      telemetry.addData("bitmap", "pixels copied from image"); //RGB565
      telemetry.update();
      sleep(5000);

      // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
      //Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC3);
      Mat mat = new Mat();
      telemetry.addData("mat", "created unsimilar mat"); //RGB565
      telemetry.update();
      sleep(5000);
      Utils.bitmapToMat(bm, mat);
      telemetry.addData("mat", "bitmap converted to mat"); //RGB565
      telemetry.update();
      sleep(5000);

      // convert to grayscale before returning
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
      telemetry.addData("mat", "mat converted to grayscale"); //RGB565
      telemetry.update();
      sleep(5000);

      frame.close();









      telemetry.addData("Status", "frames");
      telemetry.update();
      sleep(2000);
      //Mat threshed = MM_OpenCV.Threshold(frame, 205);
      telemetry.addData("Status", "threshed");
      telemetry.update();
      //Mat morphed = MM_OpenCV.Morphology(threshed);
      telemetry.addData("Status", "morphed");
      telemetry.update();
      //List<MatOfPoint> contours = MM_OpenCV.findContours(morphed);
      telemetry.addData("Really?", "contoured");
      telemetry.update();
      while (opModeIsActive()){
          idle();
      }

  }


}


