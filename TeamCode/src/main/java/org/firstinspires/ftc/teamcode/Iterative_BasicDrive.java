package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MM_Classes.MM_IterativeOpMode;
@Disabled
@TeleOp(name = "Iterative Drive")
public class Iterative_BasicDrive extends MM_IterativeOpMode {
    @Override
    public void init() {
        super.init();
    }
    Boolean safety = true;
    @Override
    public void loop(){
        double scalar;
        if (gamepad1.b) {
            scalar = 1;
        }
        else{
            scalar = .35;
        }
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * scalar;
        double radians = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double turn = gamepad1.right_stick_x * scalar;
        final double v1 = r * Math.cos(radians) - turn;
        final double v2 = r * Math.sin(radians) + turn;
        final double v3 = r * Math.sin(radians) - turn;
        final double v4 = r * Math.cos(radians) + turn;
        robot.motorPowers(v1, v2, v3, v4);
        telemetry.addData("Steering","Mag: %.2f, Theta: %.1f deg", r, (radians*180)/Math.PI);
        telemetry.addData("Motor Powers", "fl: %.3f, fr: %.3f, br: %.3f, bl: %.3f", v1,v2,v3,v4);
        telemetry.addData("Scalar value", scalar);
    }
}
