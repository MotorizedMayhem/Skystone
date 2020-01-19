/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * Display patterns of a REV Robotics Blinkin LED Driver.
 * AUTO mode cycles through all of the patterns.
 * MANUAL mode allows the user to manually change patterns using the
 * left and right bumpers of a gamepad.
 *
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
@TeleOp(name="SaturdayScrimage")
@Disabled
public class MainDriverControl extends OpMode {

    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 2;
    private DcMotor flDrive = null;
    private DcMotor frDrive = null;
    private DcMotor blDrive = null;
    private DcMotor brDrive = null;
    private DcMotor lift = null;
    private DcMotor extend = null;
    private BNO055IMU imuBase = null;
    double LiftPosit;
    double ExtendPosit;
    boolean QuarryAttack = false;
    double SafeLift = -725;
    double SafeExtend = -1025;
    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item  patternName;
    Telemetry.Item  display;
    DisplayKind     displayKind;
    Deadline        ledCycleDeadline;
    Deadline        gamepadRateLimit;
    Orientation     angles;
    Servo           MServo;
    Servo           LServo;
    Servo           RServo;
    Servo           FServo;


    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    @Override
    public void init()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = false;
        parameters.loggingTag           = "IMU";

        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuBase = hardwareMap.get(BNO055IMU.class, "imu_base");
        imuBase.initialize(parameters);

        LServo = hardwareMap.servo.get("Lservo");
        MServo = hardwareMap.servo.get("Mservo");
        RServo = hardwareMap.servo.get("Rservo");
        FServo = hardwareMap.servo.get("servo");

        displayKind = DisplayKind.MANUAL;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "Blinky");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        flDrive = hardwareMap.get(DcMotor.class, "fl");
        frDrive = hardwareMap.get(DcMotor.class, "fr");
        blDrive = hardwareMap.get(DcMotor.class, "bl");
        brDrive = hardwareMap.get(DcMotor.class, "br");
        lift    = hardwareMap.get(DcMotor.class, "lift");
        extend  = hardwareMap.get(DcMotor.class, "extend");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        brDrive.setDirection(DcMotor.Direction.FORWARD);

        extend.setDirection(DcMotor.Direction.FORWARD);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftPosit   = lift.getCurrentPosition();
        ExtendPosit = extend.getCurrentPosition();
    }

    boolean lightStat = false;
    @Override
    public void loop()
    {
        if (gamepad1.start){
            telemetry.addData("Controller", "1 has start");
        }
        if (gamepad1.start){
            telemetry.addData("Controller", "2 has start");
        }

        double flPower;
        double frPower;
        double blPower;
        double brPower;
        boolean Eup     = gamepad2.dpad_left;
        boolean Edown   = gamepad2.dpad_right;
        boolean Lup     = gamepad2.dpad_up;
        boolean Ldown   = gamepad2.dpad_down;

        handleGamepad();

        if (displayKind == DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
             */
        }

        boolean Long     = gamepad2.x;
        boolean Short    = gamepad2.b;
        boolean Attack   = gamepad2.a;
        boolean Collapse = gamepad2.y;

        // LServo is Front
        // MServo is Middle
        // RServo is Back (adjacent to elevator)

        // Position of 0 is rotation towards elevator
        // Position of 1 is rotation towards front of robot
        if (Short) {
            LServo.setPosition(0.3);
            MServo.setPosition(0.5);
            RServo.setPosition(1);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        }
        else if (Long) {
            LServo.setPosition(0.3);
            MServo.setPosition(0);
            RServo.setPosition(1);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (Attack) {
            LServo.setPosition(1);
            MServo.setPosition(0.5);
            RServo.setPosition(1);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (Collapse) {
            LServo.setPosition(0);
            MServo.setPosition(0);
            RServo.setPosition(1);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }

        boolean FDown = gamepad1.y;
        boolean FUp   = gamepad1.a;

        if (FDown){
            FServo.setPosition(1);

        }
        if (FUp){
            FServo.setPosition(0);

        }
        int iterateAmnt = 100;
        if (gamepad2.right_bumper){
            iterateAmnt /= 4;
        }
        if (gamepad1.dpad_up){
            QuarryAttack = true;
        }
        else if (gamepad1.dpad_down){
            QuarryAttack = false;
            LServo.setPosition(0);
            MServo.setPosition(0);
            RServo.setPosition(1);

        }
        if (QuarryAttack){
            ExtendPosit = -2250;
            LiftPosit = -525;
            if (lift.getCurrentPosition() < SafeLift || extend.getCurrentPosition() < SafeExtend){
                LServo.setPosition(1);
                MServo.setPosition(0.5);
                RServo.setPosition(1);
            }
        }
        // if (false){
        //     ExtendPosit = -2450;
        //     LiftPosit = 0;
        //     while (LiftPosit < 100){
        //     LServo.setPosition(1);
        //     MServo.setPosition(0.5);
        //     RServo.setPosition(1);
        //     }

        // }
        if (gamepad1.dpad_left){
            LServo.setPosition(0.3);
            MServo.setPosition(0);
            RServo.setPosition(1);
            ExtendPosit = -2450;
            LiftPosit = -5400;
        }
        if (gamepad1.dpad_right){
            LServo.setPosition(0.3);
            MServo.setPosition(0.5);
            RServo.setPosition(1);
            ExtendPosit = -2450;
            LiftPosit = -5400;
        }

        if (Eup) {
            ExtendPosit = Range.clip(ExtendPosit + iterateAmnt, -4900, 0);
        }
        if (Edown) {
            ExtendPosit = Range.clip(ExtendPosit - iterateAmnt, -4900, 0);
        }
        if (Lup) {
            LiftPosit = Range.clip(LiftPosit - iterateAmnt, -5400, 0);
        }
        if (Ldown) {
            LiftPosit = Range.clip(LiftPosit + iterateAmnt, -5400, 0);
        }


        if (lift.getCurrentPosition() < -5300 || extend.getCurrentPosition() < -4800)
        {
            lightStat = true;
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);

        }
        else if(lightStat)
        {
            lightStat = false;
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);

        }

        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition((int)LiftPosit);

        extend.setPower(1);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition((int)ExtendPosit);

        telemetry.addData("Status", "LiftPosit: " + LiftPosit);
        telemetry.addData("Status", "CurrentLiftPos: " + lift.getCurrentPosition());
        telemetry.addData("Status", "ExtendPosit: " + ExtendPosit);
        telemetry.addData("Status", "CurrentExtendPos: " + extend.getCurrentPosition());

        angles   = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double imu_yaw = angles.firstAngle;

        double r =Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if (!gamepad1.left_bumper)
        {
            r *= .5;
        }
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 + imu_yaw;
        double rightX = -gamepad1.right_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
        // Send calculated power to wheels
        flDrive.setPower(v1 );
        frDrive.setPower(v2 );
        blDrive.setPower(v3 );
        brDrive.setPower(v4 );
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */

    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            setDisplayKind(DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            setDisplayKind(DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            for (int i = (int)(Math.random()*100.0); i>0; i--){
                pattern = pattern.next();
            }
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}
