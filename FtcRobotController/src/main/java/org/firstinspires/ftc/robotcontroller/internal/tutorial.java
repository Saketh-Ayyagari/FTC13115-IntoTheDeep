package org.firstinspires.ftc.robotcontroller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FTC13115")
public class tutorial extends OpMode
{
    DcMotor motor;
    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotor.class, ""); // add motor name from the app or control hub in those empty quotes
        telemetry.addData("Hardware", "Initialized");

    }
    @Override
    public void loop()
    {
        float x = gamepad1.left_stick_x;
        if (gamepad1.left_stick_x > 0)
        {
            motor.setPower(0.5);
        }
        if (gamepad1.right_stick_x > 0)
        {
            motor.setDirection(x); // have it so that adjusting the right stick changes the direction of motor
        }
        motor.setPower(0);
    }
}