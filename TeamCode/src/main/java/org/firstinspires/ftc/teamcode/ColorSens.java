package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSens extends OpMode
{
    // i had a problem; it was  that  i called ColorSensor "Colorsens"
    ColorSensor colorSens;

    @Override
    public void init()
    {
        colorSens = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    @Override
    public void loop()
    {
        int red = colorSens.red();
        int green = colorSens.green();
        int blue = colorSens.blue();

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Display the dominant color
        if (red > green && red > blue) {
            telemetry.addData("Detected Color", "Red");
        }
        else if (green > red && green > blue) {
            telemetry.addData("Detected Color", "Green");
        }
        else if (blue > red && blue > green) {
            telemetry.addData("Detected Color", "Blue");
        }
        else {
            telemetry.addData("Detected Color", "Unknown");
        }

        telemetry.update();
    }
}