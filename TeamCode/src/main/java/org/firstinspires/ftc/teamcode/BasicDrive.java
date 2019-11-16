package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;

@Disabled
@TeleOp(name = "Basic Drive")
public class BasicDrive extends LinearOpMode {
    MecanumYellow robot = new MecanumYellow();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("robot", "started");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * .4;
            double radians = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double turn = -gamepad1.right_stick_x * .4;
            final double v1 = r * Math.cos(radians) - turn;
            final double v2 = r * Math.sin(radians) + turn;
            final double v3 = r * Math.sin(radians) - turn;
            final double v4 = r * Math.cos(radians) + turn;
            robot.motor_powers(v1, v2, v3, v4);
        }

    }
}
