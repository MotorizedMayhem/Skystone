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

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MM_Classes.MM_Vuforia;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Vuforia Test", group = "Computer Vision")
@Disabled
public class VuforiaTests extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  //private MecanumChassis2019 robot = new MecanumChassis2019();
  private MM_Vuforia vuforia = new MM_Vuforia();

  private float[] position = new float[]{0,0,0};
  private float[] rotation = new float[]{0,0,0};


    @Override
  public void init() {
    telemetry.addData("Status", "Starting Robot");
    telemetry.update();
    //robot.init(hardwareMap);
    telemetry.addData("Status", "Starting Camera");
    telemetry.update();
    vuforia.init(hardwareMap, MM_Vuforia.USE_SCREEN, MM_Vuforia.USE_PHONECAM);
    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    Pair<OpenGLMatrix,String> pairReturn =  vuforia.scanTargets();
    OpenGLMatrix pose = pairReturn.first;
    String name = pairReturn.second;
    if (pose != null) {
        position = MM_Vuforia.getXYZ(pose, DistanceUnit.INCH);
        rotation = MM_Vuforia.getPitchRollYaw(pose, AngleUnit.DEGREES);
    }
    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", position[0], position[1], position[2]);
    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation[0], rotation[1], rotation[2]);
    telemetry.addData("Target ID: ", name);
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }
}
