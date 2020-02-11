package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.teamcode.MM_Classes.MecanumChassis2019;


@TeleOp(name="KodaKids", group="Iterative Opmode")
@Disabled
public class KodaKids extends OpMode
{
    // Declare OpMode members.

    private final static int LED_PERIOD = 1;
    private MecanumChassis2019 robot = new MecanumChassis2019();
    private boolean goldFound;      // Sound file present flags
    private boolean silverFound;
    private boolean wasPressed = false;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;
    private ElapsedTime runtime = new ElapsedTime();

    private RevBlinkinLedDriver.BlinkinPattern touched = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern notTouched = RevBlinkinLedDriver.BlinkinPattern.RED;

    private int silverSoundID, goldSoundID;
    private AndroidTextToSpeech t2s;

    @Override
    public void init() {
        silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
        t2s = new AndroidTextToSpeech();
        t2s.initialize();
        t2s.setLanguageAndCountry("en","US");
        telemetry.addData("Status", "Initializing");
        robot.init(hardwareMap);
        telemetry.addData("Status","Robot Done; starting sounds");
        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);
        telemetry.addData("Status", "Initialized");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

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
    double PowerPush = 0.4;

    @Override
    public void loop() {
        boolean slow = !gamepad1.left_bumper; //true if not pressed
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double r =Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if (slow){r *= PowerPush;}
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        final double v1 = r * Math.sin(robotAngle) + rightX;
        final double v2 = r * Math.cos(robotAngle) - rightX;
        final double v3 = r * Math.cos(robotAngle) + rightX;
        final double v4 = r * Math.sin(robotAngle) - rightX;

        if (!robot.digitalTouch.getState()){ //if its touched (down)
            robot.setLights(touched);
            if(wasPressed == false && silverFound) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                //t2s.speak("testing one two three");
                wasPressed = true;
            }
            //telemetry.addData("touched","true");
        }
        else{
            robot.setLights(notTouched);
            wasPressed = false;
        }

        robot.motor_powers(v1,v2,v3,v4);
        telemetry.addData("touched",!robot.digitalTouch.getState());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "PowPush: " + PowerPush);
    }
    @Override
    public void stop(){
        t2s.close();
    }

}
