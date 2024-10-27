package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="AutoJ")
public class AutoJ extends Robot
{

    public AutoJ(HardwareMap hwMp)
    {
        super(hwMp);
    }

    public void init(){
        super.init();
    }

    public void powerMotors()
    {
        super.powerMotors(0.5, 0.5, 0.5);
    }

    public void moveRobotEncoders()
    {
        super.moveRobotEncoders(3, 1, 0.5);
    }
}

