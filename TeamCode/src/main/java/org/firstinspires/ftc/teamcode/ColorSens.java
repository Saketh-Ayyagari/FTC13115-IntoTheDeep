package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSens
{
    ColorSens colorSens;

    @Override
    public void init()
    {
        colorSens = hardwareMap.get(ColorSens.class, "colorsensor");
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
        } else if (green > red && green > blue) {
            telemetry.addData("Detected Color", "Green");
        } else if (blue > red && blue > green) {
            telemetry.addData("Detected Color", "Blue");
        } else {
            telemetry.addData("Detected Color", "Unknown");
        }

        telemetry.update();
    }

    }
}
