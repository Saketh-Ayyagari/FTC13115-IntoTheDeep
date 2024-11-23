package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Saketh Ayyagari
 * "Robot" class for motor control
 * READ BEFORE MODIFYING: any mention of left, right, back, etc. are from the perspective of behind the
 * robot.
 */
public class Robot{
    private HardwareMap hardwareMp; // initializing motors/sensors;
    // Declare OpMode members for each of the chassis motors.
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    // motors for slide and intake control
    public DcMotor slide;
    public Servo extend, left, right;

    // maximum power robot can drive
    public double MAX_POWER;

    private Telemetry telemetry; // for FTC dashboard--will integrate later


    // initializes robot motors, encoders, etc. MUST be run before any movement occurs
    // the init method must be the one to take in a
    public Robot(double max_power){
        this.MAX_POWER = max_power;
    }
    public void init(HardwareMap hwMp){
        hardwareMp = hwMp;
        // initializes all motors
        backRight = hardwareMp.get(DcMotor.class, "backRight"); //port 3
        frontRight = hardwareMp.get(DcMotor.class, "frontRight"); //port 2
        backLeft = hardwareMp.get(DcMotor.class, "backLeft"); //port 1
        frontLeft = hardwareMp.get(DcMotor.class, "frontLeft"); //port 0

        slide = hardwareMp.get(DcMotor.class, "slide"); // port ___ on Expansion Hub


        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        //slide.setDirection(DcMotor.Direction.REVERSE);

        // setting the mode of each motor to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //start at 0 power
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
    * Positive power = rotate counterclockwise (causes heading to become larger)
    * Negative power = rotate clockwise (causes heading to become smaller)
    * Just like unit circle
    * */
    public void rotate(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    /*
     * Negative Power = strafe left
     * Positive power = strafe right
     * */
    public void brake(){
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    // given parameters for drive, rotation, and strafe power, send power to the motors
    public void powerMotors(double drive, double turn, double strafe){
        double leftPower = Range.clip(drive - turn, -MAX_POWER, MAX_POWER);
        double rightPower = Range.clip(drive + turn, -MAX_POWER, MAX_POWER);

        // Send calculated power to wheels
        frontLeft.setPower(Range.clip(leftPower-strafe, -MAX_POWER, MAX_POWER));
        frontRight.setPower(Range.clip(rightPower+strafe, -MAX_POWER, MAX_POWER));
        backLeft.setPower(Range.clip(leftPower+strafe, -MAX_POWER, MAX_POWER));
        backRight.setPower(Range.clip(rightPower-strafe, -MAX_POWER, MAX_POWER));
    }

    /**
     * Uses encoders to move in specific directions
     * forward = moving forward
     * backward = moving backward
     * counterclockwise = moving counterclockwise
     * clockwise = moving clockwise
     */
    public void moveRobotwEncoders(String direction, double inches){
        // Reset the encoder
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double     COUNTS_PER_MOTOR_REV    = 2786.2 ;
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;
        double     WHEEL_DIAMETER_IN   = 4.09449;
        double     COUNTS_PER_IN         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_IN * Math.PI);
        double     DRIVE_SPEED             = 0.6;
        double     TURN_SPEED              = 0.5;

        double target = inches * COUNTS_PER_IN; // may have to find a way to convert from inches to ticks

        double leftPower = this.MAX_POWER;
        double rightPower = this.MAX_POWER;

        // different scenarios based on direction specified
        switch(direction){
            case "forward":
                frontLeft.setTargetPosition((int)target);
                backLeft.setTargetPosition((int)target);
                frontRight.setTargetPosition((int)target);
                backRight.setTargetPosition((int)target);
            case "backward":
                frontLeft.setTargetPosition(-(int) target);
                backLeft.setTargetPosition(-(int) target);
                frontRight.setTargetPosition(-(int) target);
                backRight.setTargetPosition(-(int) target);
                
                leftPower *= -1;
                rightPower *= -1;
            case "counterclockwise":
                frontLeft.setTargetPosition(-(int)target);
                backLeft.setTargetPosition(-(int)target);
                frontRight.setTargetPosition((int)target);
                backRight.setTargetPosition((int)target);
                
                leftPower *= -1;
            case "clockwise":
                frontLeft.setTargetPosition((int)target);
                backLeft.setTargetPosition((int)target);
                frontRight.setTargetPosition(-(int)target);
                backRight.setTargetPosition(-(int)target);
                
                rightPower *= -1;

        }

        // Set the motor to run to the target position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the motor power
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }
}

