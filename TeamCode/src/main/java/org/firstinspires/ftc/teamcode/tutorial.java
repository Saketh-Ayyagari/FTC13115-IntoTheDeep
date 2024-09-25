package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FTC13115")

public class tutorial extends OpMode // class name must be the same as file name
{
    DcMotor motor;
    final double ticks = 537.6;
    double newTarget;

    @Override
    public void init()
    {
        motor = hardwareMap.get(DcMotor.class, ""); // add device name from motors in control hub
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop()
    {
        float x = gamepad1.left_stick_x;
        if (gamepad1.left_stick_x > 0)
        {
            motor.setPower(x);
            encoder(2);
        }
        motor.setPower(0); // turns off the motor
        telemetry.addData("Motor ticks: ", motor.getCurrentPosition());

    }
    public void encoder(int turnage)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks/turnage;
        motor.setTargetPosition((int)newTarget);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker() {
        motor.setTargetPosition(0);
        motor.setPower(0.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
