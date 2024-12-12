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

    private final double MAX_POWER = 0.7;
    /*git
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
        // TURNING TEMPORARILY DISABLED -- 11/25
        double drive = gamepad1.left_stick_y; // moving forward or backward
        double turn = gamepad1.right_stick_x; // strafing left or right
        double strafe = gamepad1.left_stick_x; // turning clockwise or counterclockwise
        double lift = gamepad1.left_trigger - gamepad1.right_trigger; // lifting the slide

        drivetrain.powerChassisMotors(drive, turn, strafe); // sends individual powers to the motors
        drivetrain.liftSlide(lift);

        if (gamepad1.right_bumper){
            drivetrain.roll_in();
        }
        if (gamepad1.left_bumper){
            drivetrain.roll_out();
        }
        if (gamepad1.a){
            drivetrain.liftServo(0.3);
        }
        if (gamepad1.b){
            drivetrain.liftServo(0.13);
        }
        if (gamepad1.x){
            drivetrain.liftServo(0.5);
        }
        else{
            drivetrain.stop_intake();
        }

    }
}
