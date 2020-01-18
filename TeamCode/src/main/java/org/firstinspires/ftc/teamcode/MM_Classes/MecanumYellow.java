package org.firstinspires.ftc.teamcode.MM_Classes;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumYellow {
    private final static int LED_PERIOD = 1;

    //Rate limit gamepad button presses to every 500ms
    private final static int GAMEPAD_LOCKOUT = 500;

    private boolean closedLoopEncoder;

    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;
    public DcMotor lift;
    public DcMotor extend;
    public double LiftPosit;
    public double ExtendPosit;

    Servo           MServo;
    Servo           LServo;
    Servo           RServo;
    Servo           FServo;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    //Dual Imu
    public BNO055IMU imuBase;
    public BNO055IMU imuTop;
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    //Used for Color Sensor
    public NormalizedColorSensor colorLeft;
    public NormalizedColorSensor colorRight;


    private HardwareMap hardwareMap;

    public MecanumYellow(boolean closedLoopEncoder) {
        this.closedLoopEncoder = closedLoopEncoder;
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
        //parameters.accelerationIntegrationAlgorithm = new MM_AccelerationIntegrator();

        imuBase = hardwareMap.get(BNO055IMU.class, "imu_base");
        imuTop = hardwareMap.get(BNO055IMU.class, "imu_top");
        imuBase.initialize(parameters);
        imuTop.initialize(parameters);

        LServo = hardwareMap.servo.get("Lservo");
        MServo = hardwareMap.servo.get("Mservo");
        RServo = hardwareMap.servo.get("Rservo");
        FServo = hardwareMap.servo.get("servo");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "Blinky");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);




        flDrive = hardwareMap.get(DcMotor.class, "fl");
        frDrive = hardwareMap.get(DcMotor.class, "fr");
        blDrive = hardwareMap.get(DcMotor.class, "bl");
        brDrive = hardwareMap.get(DcMotor.class, "br");
        lift    = hardwareMap.get(DcMotor.class, "lift");
        extend  = hardwareMap.get(DcMotor.class, "extend");
        //channel end forward

        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        brDrive.setDirection(DcMotor.Direction.FORWARD);


        //plate end forward
        /*
        flDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        if (closedLoopEncoder){
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

        extend.setDirection(DcMotor.Direction.FORWARD);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftPosit   = lift.getCurrentPosition();
        ExtendPosit = extend.getCurrentPosition();

        colorLeft= hardwareMap.get(NormalizedColorSensor.class, "color_left");
        colorRight=hardwareMap.get(NormalizedColorSensor.class, "color_right");
    }


    public void motorPowers(double power){
        motorPowers(power,power,power,power);
    }
    public void motorPowers(double fl, double fr, double bl, double br){
        flDrive.setPower(fl);
        frDrive.setPower(fr);
        blDrive.setPower(bl);
        brDrive.setPower(br);
    }

    public final static int
        COUNTERCLOCKWISE = -1,
        CLOCKWISE = 1;

    public void rotate(double power, int direction){
        double left = power * direction;
        double right = power * -direction;
        motorPowers(left,right,left,right);
    }
    public void stopMotors(){
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
    }

    public double[] getColor(NormalizedColorSensor colorSensor) //returns in rgba
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return new double[]{colors.red,colors.green,colors.blue,colors.alpha};
    }

    public double[] getColorHSV(NormalizedColorSensor colorSensor) //returns in rgba
    {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return new double[]{hsvValues[0],hsvValues[1],hsvValues[2]};
    }

    public Orientation getIMU_ZYX(AngleUnit unit){ //heading first
        Orientation angles = imuTop.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit);
        return angles;
    }

    public double getIMU_Heading(AngleUnit unit){
        return getIMU_ZYX(unit).firstAngle;
    }

    public final static double
        LEFT = 180,
        FORWARD = 90,
        RIGHT = 0,
        BACKWARD = 270;

    public void vectorDrive(double r, double degrees){ //degrees corresponds to a normal unit circle
        double radians = (degrees * Math.PI)/ 180 - Math.PI/4;
        //r *= 1.25;
        final double v1 = r * Math.cos(radians);
        final double v2 = r * Math.sin(radians);
        final double v3 = r * Math.sin(radians);
        final double v4 = r * Math.cos(radians);
        motorPowers(v1,v2,v3,v4);
    }
    public void resetEncoders(){
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (closedLoopEncoder) {
            flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            //should be a wait but whatever
            frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void runToPositions(int fl,int fr, int bl, int br){
        flDrive.setTargetPosition(fl);
        frDrive.setTargetPosition(fr);
        blDrive.setTargetPosition(bl);
        brDrive.setTargetPosition(br);
        
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void runToPositions(int distance){
        runToPositions(distance,distance,distance,distance);
    }

    public double encoderAvg(){
        double total = Math.abs(frDrive.getCurrentPosition()) + Math.abs(flDrive.getCurrentPosition())
                + Math.abs(brDrive.getCurrentPosition()) + Math.abs(blDrive.getCurrentPosition());
        return total/4;
    }

    public boolean motorsAreBusy(){
        return flDrive.isBusy() || frDrive.isBusy() || blDrive.isBusy() || brDrive.isBusy();
    }

}
