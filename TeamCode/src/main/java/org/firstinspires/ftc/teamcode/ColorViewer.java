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
        double[] colorRGBA = robot.getColor(robot.colorLeft);
        double[] colorHSV = robot.getColorHSV(robot.colorLeft);
        telemetry.addData("RGBA Left","R: %.2f, G: %.2f, B: %.2f, A: %.2f",
                colorRGBA[0], colorRGBA[1], colorRGBA[2], colorRGBA[3]);
        telemetry.addData("HSV Left","H: %.2f, S: %.2f, V: %.2f",
                colorHSV[0], colorHSV[1], colorHSV[2]);

        colorRGBA = robot.getColor(robot.colorRight);
        colorHSV = robot.getColorHSV(robot.colorRight);
        telemetry.addData("RGBA Right","R: %.2f, G: %.2f, B: %.2f, A: %.2f",
                colorRGBA[0], colorRGBA[1], colorRGBA[2], colorRGBA[3]);
        telemetry.addData("HSV Right","H: %.2f, S: %.2f, V: %.2f",
                colorHSV[0], colorHSV[1], colorHSV[2]);


    }
}
