package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class FinalTeleopNEW extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    private Robot drivetrain = new Robot(0.6);
    private double MAX_POWER = 0.6;
    // motors for slide and intake control
    public DcMotor slide;
    public Servo extend;
    public Servo left, right;

    public final double open_pos = 0;
    public final double closed_pos = 0.45;
    // variables for closed-loop slide control
    private final PIDController slideControl = new PIDController(0.03125); // feedforward constant for holding slide up
    private int setpoint_slide;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        slide = hardwareMap.get(DcMotor.class, "slide"); // port ___ on Expansion Hub

        //slide = hardwareMap.dcMotor.get( "slide"); // port ___ on Expansion Hub
        extend = hardwareMap.get(Servo.class, "extend");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y; // moving forward or backward
            double turn = gamepad1.right_stick_x; // strafing left or right
            double strafe = gamepad1.left_stick_x; // turning clockwise or counterclockwise
            double lift = gamepad1.left_trigger - gamepad1.right_trigger; // lifting the slide

            // calculates power of individual motors given drive, turn, and strafe values.
            double frontLeftPower, frontRightPower, backRightPower, backLeftPower;

            frontLeftPower = drive + turn + strafe;
            backLeftPower = drive + turn - strafe;
            frontRightPower = drive - turn - strafe;
            backRightPower = drive - turn + strafe;

            if (gamepad1.left_bumper){
                close();
            }
            else if (gamepad1.right_bumper){
                open();
            }
            if (gamepad1.a){
                liftServo(0);
            }
            if (gamepad1.b){
                liftServo(0.33);
            }

            /*
             * Send calculated power to wheels
             * strafing happens independently on each wheel
             * Range.clip() clamps the power sent to each motor between -MAX_POWER and MAX_POWER
             */
            frontLeft.setPower(Range.clip(frontLeftPower, -MAX_POWER, MAX_POWER));
            frontRight.setPower(Range.clip(frontRightPower, -MAX_POWER, MAX_POWER));
            backLeft.setPower(Range.clip(backLeftPower, -MAX_POWER, MAX_POWER));
            backRight.setPower(Range.clip(backRightPower, -MAX_POWER, MAX_POWER));
            slide.setPower(lift);
        }
    }

    // lifts extender to change orientation of the sample grip
    public void liftServo(double position){
        extend.setPosition(position);
    }
    // sends power directly to slide for teleop
    public void liftSlide(double power){
        if (power != 0){
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(power);
            setpoint_slide = slide.getCurrentPosition();
        }
        else{
            double power_hold = slideControl.update(setpoint_slide, slide.getCurrentPosition());
            slide.setPower(power_hold);
        }
    }
    // rotates pulley a certain number of rotations to lift the slide
    // two cases: up and down
    public void liftSlide(double inches, String dir){
        final double TICKS_PER_REV = 537.7; // ticks per revolution
        final double INCHES_PER_REV = 120/25.4; // for every 1 rotation, the belt moves 120 MM

        int target = (int)(((TICKS_PER_REV)/(INCHES_PER_REV)) * inches);

        double power = MAX_POWER;
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (dir.equals("up")){
            //go down target ticks
            //neg power neg target
            power *= -1;
            target *= -1;
        }
        slide.setTargetPosition(target);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        while (slide.isBusy()){

        }
        int error = target - slide.getCurrentPosition();
        // constant power sent to slide motor from there
        slide.setPower(Range.clip(slideControl.update(target,
                slide.getCurrentPosition()), -1, 1));
    }
    // SERVO RANGE FROM 0 - 1
    public void open(){
        left.setPosition(open_pos);
        right.setPosition(open_pos);
    }
    public void close(){
        left.setPosition(closed_pos);
        right.setPosition(closed_pos);
    }

}
