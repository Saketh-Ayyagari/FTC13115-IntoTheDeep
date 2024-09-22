package org.firstinspires.ftc.robotcontroller.internal;
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
        motor = hardwareMap.get(DcMotor.class, ""); // add device name from motors in control hub
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
    }
    @Override
    public void loop()
    {
        float x = gamepad1.left_stick_x;
        if (gamepad1.left_stick_x > 0)
        {
            motor.setPower(x);
        }
        motor.setPower(0); // turns off the motor
    }
}
