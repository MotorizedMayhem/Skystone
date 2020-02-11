package org.firstinspires.ftc.teamcode.MM_Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BarebonesYellow {
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

    private HardwareMap hardwareMap;

    public BarebonesYellow(boolean closedLoopEncoder) {
        this.closedLoopEncoder = closedLoopEncoder;
    }
    public BarebonesYellow(){this(false);} //default dont use them


    public void init(HardwareMap hwMap){
        hardwareMap = hwMap;

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
    }


    public void motor_powers(double power){
        motor_powers(power,power,power,power);
    }
    public void motor_powers(double fl, double fr, double bl, double br){
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
        motor_powers(left,right,left,right);
    }
    public void stop_motors(){
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
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
        motor_powers(v1,v2,v3,v4);
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
