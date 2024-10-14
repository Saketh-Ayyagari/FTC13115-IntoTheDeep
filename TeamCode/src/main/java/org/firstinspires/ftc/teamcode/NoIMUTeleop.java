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
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    private final double MAX_POWER = 0.8;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        backRight = hardwareMap.get(DcMotor.class, "backRight"); //port 3
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //port 2
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); //port 1
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); //port 0

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        // setting the mode of each motor to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //start at 0 power
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //Flipped x and y because motors are flipped - 12/16
        double drive = gamepad1.left_stick_y; //controls drive by moving up or down.
        //
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        telemetry.addData("Drive Power: ", drive);
        telemetry.addData("Turning Value: ", turn);
        telemetry.addData("Strafing Value: ", strafe);
        telemetry.addLine();

        leftPower = Range.clip(drive - turn, -MAX_POWER, MAX_POWER);
        rightPower = Range.clip(drive + turn, -MAX_POWER, MAX_POWER);

        // Send calculated power to wheels
        frontLeft.setPower(Range.clip(leftPower-strafe, -MAX_POWER, MAX_POWER));
        frontRight.setPower(Range.clip(rightPower+strafe, -MAX_POWER, MAX_POWER));
        backLeft.setPower(Range.clip(leftPower+strafe, -MAX_POWER, MAX_POWER));
        backRight.setPower(Range.clip(rightPower-strafe, -MAX_POWER, MAX_POWER));


    }
}
