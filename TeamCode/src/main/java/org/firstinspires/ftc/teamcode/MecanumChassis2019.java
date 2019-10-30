package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MecanumChassis2019 {
    private final static int LED_PERIOD = 1;

    //Rate limit gamepad button presses to every 500ms
    private final static int GAMEPAD_LOCKOUT = 500;

    DcMotor flDrive;
    DcMotor frDrive;
    DcMotor blDrive;
    DcMotor brDrive;

    //Dual Imu
    BNO055IMU imuBase;
    BNO055IMU imuTurn;
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //Used for touch Sensor
    DigitalChannel digitalTouch;

    //Used for Color Sensor
    NormalizedColorSensor colorLeft;
    NormalizedColorSensor colorRight;

    //Light Strip
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private HardwareMap hardwareMap;

    MecanumChassis2019() { }


    void init(HardwareMap hwMap){
        hardwareMap = hwMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuBase = hardwareMap.get(BNO055IMU.class, "imu_base");
        imuTurn = hardwareMap.get(BNO055IMU.class, "imu_base");
        imuBase.initialize(parameters);
        imuTurn.initialize(parameters);


        flDrive = hardwareMap.get(DcMotor.class, "fl");
        frDrive = hardwareMap.get(DcMotor.class, "fr");
        blDrive = hardwareMap.get(DcMotor.class, "bl");
        brDrive = hardwareMap.get(DcMotor.class, "br");
        flDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.REVERSE);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        colorLeft= hardwareMap.get(NormalizedColorSensor.class, "color_left");
        colorRight=hardwareMap.get(NormalizedColorSensor.class, "color_right");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "Blinky");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }


    void motor_powers(double power){
        motor_powers(power,power,power,power);
    }
    void motor_powers(double fl, double fr, double bl, double br){
        flDrive.setPower(fl);
        frDrive.setPower(fr);
        blDrive.setPower(bl);
        brDrive.setPower(br);
    }
    void stop_motors(){
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
    }

    double[] getColorLeft() //returns in rgba
    {
        NormalizedRGBA colors = colorLeft.getNormalizedColors();
        return new double[]{colors.red,colors.green,colors.blue,colors.alpha};
    }
    double[] getColorRight() //returns in rgba
    {
        NormalizedRGBA colors = colorRight.getNormalizedColors();
        return new double[]{colors.red,colors.green,colors.blue,colors.alpha};
    }

    double[] getColorLeftHSV() //returns in rgba
    {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorLeft.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return new double[]{hsvValues[0],hsvValues[1],hsvValues[2]};
    }
    
    double[] getColorRightHSV() //returns in rgba
    {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorRight.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return new double[]{hsvValues[0],hsvValues[1],hsvValues[2]};
    }

    void setLights(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriver.setPattern(pattern);
    }

    Orientation getIMU_ZYX(AngleUnit unit){
        Orientation angles = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit);
        return angles;
    }

}
