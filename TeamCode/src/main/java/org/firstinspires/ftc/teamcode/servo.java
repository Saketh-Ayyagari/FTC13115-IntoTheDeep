package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class servo extends OpMode
{
    public CRServo servomain;
    @Override
    public void init()
    {
        servomain = hardwareMap.get(CRServo.class, "servo");
    }

    public void loop()
    {
        if (gamepad1.b)
        {
            servo.setPower(1);
        }
    }
}
