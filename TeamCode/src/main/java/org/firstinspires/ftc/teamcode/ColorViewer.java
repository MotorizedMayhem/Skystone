package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MM_Classes.MecanumYellow;

@TeleOp(name = "Color Sensor View", group = "Sensor")
public class ColorViewer extends OpMode {
    MecanumYellow robot = new MecanumYellow();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double[] colorRGBA = robot.getColor();
        double[] colorHSV = robot.getColorHSV();
        telemetry.addData("RGBA","R: %.2f, G: %.2f, B: %.2f, A: %.2f",
                colorRGBA[0], colorRGBA[1], colorRGBA[2], colorRGBA[3]);
        telemetry.addData("HSV","H: %.2f, S: %.2f, V: %.2f",
                colorHSV[0], colorHSV[1], colorHSV[2]);
    }
}
