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

import android.graphics.drawable.GradientDrawable;
import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.EyewearDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoV1", group="Linear Opmode")
public class AutoV1 extends LinearOpMode {
    //MecanumChassis2019 robot = new MecanumChassis2019();
    MM_Vuforia vuforia  = new MM_Vuforia();
    //MM_TensorFlow tf = new MM_TensorFlow();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        ElapsedTime runtime = new ElapsedTime();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        vuforia.init(hardwareMap, MM_Vuforia.SHOW_CAMERA.USE_SCREEN);
        //tf.init(hardwareMap,80, MM_Vuforia.SHOW_CAMERA.USE_SCREEN);
        telemetry.addData("Status", "Initialized");
        telemetry.update();





        waitForStart();

        leftDrive.setPower(-.2); // go forward
        rightDrive.setPower(-.2);
        runtime.reset();
        while (runtime.milliseconds() < 1000 && opModeIsActive()) { }

        leftDrive.setPower(-.12); // go forward
        rightDrive.setPower(-.12);

        Pair<OpenGLMatrix, String> placeholder;
        placeholder = vuforia.scanTargets();
        while(placeholder.first == null && opModeIsActive()) {
            placeholder = vuforia.scanTargets();
        }
        leftDrive.setPower(0); // go forward
        rightDrive.setPower(0);
        telemetry.addData("x offset", MM_Vuforia.getXYZ(placeholder.first, DistanceUnit.MM)[0]);
        telemetry.addData("y offset", MM_Vuforia.getXYZ(placeholder.first, DistanceUnit.MM)[1]);
        telemetry.addData("z offset", MM_Vuforia.getXYZ(placeholder.first, DistanceUnit.MM)[2]);
        telemetry.update();
        while(opModeIsActive()){}


        leftDrive.setPower(-.2); // go forward
        rightDrive.setPower(-.2);
        runtime.reset();
        while (runtime.milliseconds() < 1500 && opModeIsActive()) { }


        leftDrive.setPower(-.15); // start turning
        rightDrive.setPower(.15);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (angles.firstAngle > -90 && opModeIsActive()){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        //leftDrive.setPower(0); // stop turn
        //rightDrive.setPower(0);


        leftDrive.setPower(.3); // go back
        rightDrive.setPower(.3);
        runtime.reset();
        while (runtime.milliseconds() < 1500 && opModeIsActive()) { }

        leftDrive.setPower(0); // go back
        rightDrive.setPower(0);

        runtime.reset();
        while (runtime.milliseconds() < 1000 && opModeIsActive()) { }

        leftDrive.setPower(-.15); // go forward
        rightDrive.setPower(-.15);

        /*
        String seenTarget = null;
        OpenGLMatrix position = null;
        //Pair<OpenGLMatrix, String> placeholder;
        while (seenTarget == null && opModeIsActive()){
            placeholder  = vuforia.scanTargets();
            seenTarget = placeholder.second;
            position = placeholder.first;
        }


        leftDrive.setPower(0); //stop
        rightDrive.setPower(0);

        telemetry.addData("seen", seenTarget);
        telemetry.addData("pos", MM_Vuforia.getXYZ(position, DistanceUnit.INCH));
        telemetry.update();
        */

        runtime.reset();
        while (runtime.milliseconds() < 10000 && opModeIsActive()) { }


    }
    public void sleep(int ms){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (time.milliseconds() < ms){}
    }

}
