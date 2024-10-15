package org.firstinspires.ftc.teamcode;// Use for teleop

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Saketh Ayyagari
 * Regular Teleop without IMU Assist
 */


@TeleOp(name="NoIMUTeleop", group="Iterative OpMode")
//@Disabled
public class NoIMUTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private final double MAX_POWER = 0.8;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private Robot drivetrain = new Robot(MAX_POWER);
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y; //controls drive by moving up or down.
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        drivetrain.powerMotors(drive, turn, strafe);
    }
}
