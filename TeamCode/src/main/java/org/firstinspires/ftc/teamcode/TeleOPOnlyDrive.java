package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOPOnlyDrive extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    private Robot drivetrain = new Robot(0.6);
    private double MAX_POWER = 0.6;
//    // motors for slide and intake control
//    public DcMotor slide;
//    public Servo extend;
//    public Servo left, right;
//
//    public final double open_pos = 0;
//    public final double closed_pos = 0.45;
//    // variables for closed-loop slide control
//    private final PIDController slideControl = new PIDController(0.03125); // feedforward constant for holding slide up
//    private int setpoint_slide;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        drivetrain.init(hardwareMap);

        boolean directDrive = true;

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
//        slide = hardwareMap.get(DcMotor.class, "slide"); // port ___ on Expansion Hub
//
//        //slide = hardwareMap.dcMotor.get( "slide"); // port ___ on Expansion Hub
//        extend = hardwareMap.get(Servo.class, "extend");
//        left = hardwareMap.get(Servo.class, "left");
//        right = hardwareMap.get(Servo.class, "right");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()) {
            if(directDrive) {

                telemetry.addLine("direct drive");
                telemetry.update();
                double drive = -gamepad1.left_stick_y; // moving forward or backward
                double turn = gamepad1.right_stick_x; // strafing left or right
                double strafe = gamepad1.left_stick_x; // turning clockwise or counterclockwise

                // calculates power of individual motors given drive, turn, and strafe values.
                double frontLeftPower, frontRightPower, backRightPower, backLeftPower;

                frontLeftPower = drive + turn + strafe;
                backLeftPower = drive + turn - strafe;
                frontRightPower = drive - turn - strafe;
                backRightPower = drive - turn + strafe;

                /*
                 * Send calculated power to wheels
                 * strafing happens independently on each wheel
                 * Range.clip() clamps the power sent to each motor between -MAX_POWER and MAX_POWER
                 */
                frontLeft.setPower(Range.clip(frontLeftPower, -MAX_POWER, MAX_POWER));
                frontRight.setPower(Range.clip(frontRightPower, -MAX_POWER, MAX_POWER));
                backLeft.setPower(Range.clip(backLeftPower, -MAX_POWER, MAX_POWER));
                backRight.setPower(Range.clip(backRightPower, -MAX_POWER, MAX_POWER));

                if(gamepad1.a){
                    directDrive = false;
                }
            }else{
                telemetry.addLine("robot drive");
                telemetry.update();
                double drive = -gamepad1.left_stick_y; // moving forward or backward
                double turn = gamepad1.right_stick_x; // strafing left or right
                double strafe = gamepad1.left_stick_x; // turning clockwise or counterclockwise

                drivetrain.powerChassisMotors(drive, turn, strafe);

                if(gamepad1.a){
                    directDrive = true;
                }
            }
        }
    }


}
