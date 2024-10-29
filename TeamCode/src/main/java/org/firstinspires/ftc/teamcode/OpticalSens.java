package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class OpticalSens extends OpMode {
    DistanceSensor opticalsensor; // creates a location in memory for a distance sensor
    @Override
    public void init()
    {
        //  places the location in memory as an actual distance sensor
        opticalsensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
    }

    public void loop()  // telemetry goes in the loop, must be continuously updated
    {        double value = opticalsensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance:", value);
        telemetry.update();
    }
}