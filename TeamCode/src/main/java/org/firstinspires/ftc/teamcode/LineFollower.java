
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MM_Classes.MecanumChassis2019;

@TeleOp(name="Linefollow", group="Iterative Opmode")
@Disabled
public class LineFollower extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis2019 robot = new MecanumChassis2019();

    private double LEFT_V = 0.02; //gray
    private double RIGHT_V = 0.05; //blue
    private double K_P = 15;
    private double consForward = 0.13;
    private double MIDPOINT = Math.abs(RIGHT_V+LEFT_V) / 2; //should be .035

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Left V Value", robot.getColorLeftHSV()[2]);
        telemetry.addData("Right V Value", robot.getColorRightHSV()[2]);
        telemetry.addData("Left Sum SV Value", robot.getColorLeftHSV()[1] * robot.getColorLeftHSV()[2]);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() { //follow left side of line
        double delta = MIDPOINT - robot.getColorLeftHSV()[2];
        double leftWheels = K_P * -delta + consForward;
        double rightWheels = K_P * delta + consForward;

        robot.motor_powers(leftWheels,rightWheels,leftWheels,rightWheels);

        telemetry.addData("Power Left" , leftWheels);
        telemetry.addData("Power Right" , rightWheels);
        telemetry.addData("Left V Value", robot.getColorLeftHSV()[2]);
        telemetry.addData("Right V Value", robot.getColorRightHSV()[2]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
