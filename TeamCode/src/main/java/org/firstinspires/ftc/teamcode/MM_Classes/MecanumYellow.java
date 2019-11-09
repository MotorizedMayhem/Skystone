package org.firstinspires.ftc.teamcode.MM_Classes;

import android.graphics.Color;
import android.os.ParcelFormatException;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumYellow {
    private final static int LED_PERIOD = 1;

    //Rate limit gamepad button presses to every 500ms
    private final static int GAMEPAD_LOCKOUT = 500;

    private boolean usingEncoder;

    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;

    //Dual Imu
    public BNO055IMU imu;
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //Used for Color Sensor
    public NormalizedColorSensor colorSensor;


    private HardwareMap hardwareMap;

    public MecanumYellow(boolean usingEncoder) {
        this.usingEncoder = usingEncoder;
    }
    public MecanumYellow(){this(false);} //default dont use them


    public void init(HardwareMap hwMap){
        hardwareMap = hwMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        flDrive = hardwareMap.get(DcMotor.class, "fl");
        frDrive = hardwareMap.get(DcMotor.class, "fr");
        blDrive = hardwareMap.get(DcMotor.class, "bl");
        brDrive = hardwareMap.get(DcMotor.class, "br");
        //channel end forward
        /*
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        */

        //plate end forward
        flDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.REVERSE);


        if (usingEncoder){
            flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor =hardwareMap.get(NormalizedColorSensor.class, "color");
    }


    public void motor_powers(double power){
        motor_powers(power,power,power,power);
    }
    private void motor_powers(double fl, double fr, double bl, double br){
        flDrive.setPower(fl);
        frDrive.setPower(fr);
        blDrive.setPower(bl);
        brDrive.setPower(br);
    }

    public final static int
        COUNTERCLOCKWISE = 1,
        CLOCKWISE = -1;

    public void rotate(double power, int direction){
        double left = power * direction;
        double right = power * -direction;
        motor_powers(left,right,left,right);
    }
    public void stop_motors(){
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
    }

    public double[] getColor() //returns in rgba
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return new double[]{colors.red,colors.green,colors.blue,colors.alpha};
    }

    public double[] getColorHSV() //returns in rgba
    {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return new double[]{hsvValues[0],hsvValues[1],hsvValues[2]};
    }

    public Orientation getIMU_ZYX(AngleUnit unit){ //heading first
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit);
        return angles;
    }
    public double getIMU_Heading(AngleUnit unit){
        return getIMU_ZYX(unit).firstAngle;
    }

    public void vectorDrive(double r, double degrees){
        double radians = (degrees * Math.PI)/ 180 - Math.PI/4;
        final double v1 = r * Math.cos(radians);
        final double v2 = r * Math.sin(radians);
        final double v3 = r * Math.sin(radians);
        final double v4 = r * Math.cos(radians);
        motor_powers(v1,v2,v3,v4);
    }
    public void resetEncoders(){
        if (usingEncoder) {
            frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //should be a wait but whatever
            frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double encoderAvg(){
        double total = frDrive.getCurrentPosition() + flDrive.getCurrentPosition()
                + brDrive.getCurrentPosition() + blDrive.getCurrentPosition();
        return total/4;
    }

}
