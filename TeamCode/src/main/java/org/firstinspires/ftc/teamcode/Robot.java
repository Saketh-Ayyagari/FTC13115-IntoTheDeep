package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public Servo extend;
    public Servo left, right;
    // sensor to control distance from the bottom
    public Rev2mDistanceSensor distance_slide;
    // maximum power robot can drive
    public double MAX_POWER;

    private Telemetry telemetry; // for FTC dashboard--will integrate later
    // variables for claw positions
    public final double open_pos = 0;
    public final double closed_pos = 0.45;
    // variables for closed-loop slide control
    private final PIDController slideControl = new PIDController(0.03125); // feedforward constant for holding slide up
    private int setpoint_slide;

    // initializes robot motors, encoders, etc. MUST be run before any movement occurs
    // the init method must be the one to take in a
    public Robot(double max_power){
        this.MAX_POWER = max_power;
    }
    // set maximum power the robot chassis can move
    public void setMaxPower(double new_max_power){
        this.MAX_POWER = new_max_power;
    }
    public void init(HardwareMap hwMp){
        hardwareMp = hwMp;
        // initializes all motors
        backRight = hardwareMp.get(DcMotor.class, "backRight"); //port 3
        frontRight = hardwareMp.get(DcMotor.class, "frontRight"); //port 2
        backLeft = hardwareMp.get(DcMotor.class, "backLeft"); //port 1
        frontLeft = hardwareMp.get(DcMotor.class, "frontLeft"); //port 0

        slide = hardwareMp.get(DcMotor.class, "slide"); // port ___ on Expansion Hub
        extend = hardwareMp.get(Servo.class, "extend");
        left = hardwareMp.get(Servo.class, "left");
        right = hardwareMp.get(Servo.class, "right");
        distance_slide = hardwareMp.get(Rev2mDistanceSensor.class, "slide-distance");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // setting the mode of each motor to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // resetting the encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //start at 0 power
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void brake(){
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    // given parameters for drive, rotation, and strafe power, send power to the motors
    public void powerChassisMotors(double drive, double turn, double strafe){
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
    }

    /**
     * Given a value in inches, return the number of ticks a motor must rotate.
     * NOTE THE DIMENSIONS OF YOUR WHEELS + ENCODER RESOLUTION
     * @param inches
     * @return ticks (double)
     */
    private double inchesToTicks(double inches){
        // constants for conversion from ticks to inches
        final double TICKS_PER_REV = 537.7; // encoder resolution
        final double WHEEL_DIAMETER = 96 / 25.4; // wheel diameter in inches
        final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAMETER*Math.PI);

        return TICKS_PER_IN * inches;
    }
    /**
     * Uses encoders to move in specific directions
     * forward = moving forward
     * backward = moving backward
     * counterclockwise = moving counterclockwise
     * clockwise = moving clockwise
     */
    public void moveRobotwEncoders(String direction, double inches, double speed){
        // Set the motor to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the encoder
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // converting inches to ticks
        double target = inchesToTicks(inches);

        double frontLeftPower = speed;
        double backLeftPower = speed;
        double frontRightPower = speed;
        double backRightPower = speed;

        // different scenarios based on direction specified
        switch(direction){
            case "forward":
                frontLeft.setTargetPosition((int)target);
                backLeft.setTargetPosition((int)target);
                frontRight.setTargetPosition((int)target);
                backRight.setTargetPosition((int)target);
                break;
            case "backward":
                frontLeft.setTargetPosition(-(int) target);
                backLeft.setTargetPosition(-(int) target);
                frontRight.setTargetPosition(-(int) target);
                backRight.setTargetPosition(-(int) target);

                frontLeftPower *= -1;
                backLeftPower *= -1;
                frontRightPower *= -1;
                backRightPower *= -1;

                break;
            case "left":
                frontLeft.setTargetPosition(-(int)target);
                backLeft.setTargetPosition((int)target);
                frontRight.setTargetPosition((int)target);
                backRight.setTargetPosition(-(int)target);

                frontLeftPower *= -1;
                backLeftPower *= -1;

                break;
            case "right":
                frontLeft.setTargetPosition((int)target);
                backLeft.setTargetPosition(-(int)target);
                frontRight.setTargetPosition(-(int)target);
                backRight.setTargetPosition((int)target);

                frontRightPower *= -1;
                backRightPower *= -1;

                break;
        }

        // Set the motor to run to the target position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the motor power
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Wait until the motors reach the target position
        while (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()) { // Optionally, you can add some telemetry or logging here
            //telemetry.update();
        }
        this.brake();
    }
    public void turnDegrees(double targetDegrees, String dir, double power) {
        // Set the motor to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the encoder
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double conversionTickOverMM = 1.6243;
        //distance wheels need to travel is the arc length of the circle circumscribed by the base of the robot
        //(ASSUMING THE ROBOT IS A SQUARE AND EACH WHEEL IS EQUA-DISTANT)
        //radius is the side length of the base (18 for now)/2 time sqrt2 (makes a 45 45 90 triangle)
        //then to find arc length its theta (r)
        //we get degrees so convert to radians and multiply by our radius

        double radius = 10.5; // distance from center of the robot to one of the wheels in inches
        double radians = Math.toRadians(targetDegrees*1.2);
        double target_inches = radius * radians; //arc length = inches

        /**
         * MAKE THIS A PARAMETER OF THE CLASS CONSTRUCTOR FOR REPRODUCIBILITY
         * **/
        final double TICKS_PER_REV = 537.7; // encoder resolution
        final double WHEEL_DIAMETER = 96 / 25.4; // inches
        final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAMETER*Math.PI);

        int targetPosition = (int) (target_inches * TICKS_PER_IN);

        double leftPowerLevel = power;
        double rightPowerLevel = power;
        int leftTarget = targetPosition;
        int rightTarget = targetPosition;

        if (dir.equals("clockwise")) {
            rightPowerLevel *= -1;
            rightTarget *= -1;
        } else if (dir.equals("counterclockwise")) {
            leftPowerLevel *= -1;
            leftTarget *= -1;
        }
        frontLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backLeft.setTargetPosition(leftTarget);
        backRight.setTargetPosition(rightTarget);

        // Set the motor to run to the target position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeft.setPower(leftPowerLevel);
        backLeft.setPower(leftPowerLevel);
        frontRight.setPower(rightPowerLevel);
        backRight.setPower(rightPowerLevel);

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
        right.setPosition(closed_pos);
    }
    public void close(){
        left.setPosition(closed_pos);
        right.setPosition(open_pos);
    }
}

