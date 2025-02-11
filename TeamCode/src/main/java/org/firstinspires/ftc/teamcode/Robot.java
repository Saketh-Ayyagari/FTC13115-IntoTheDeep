package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

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

    // maximum power robot can drive
    public double MAX_POWER;

    private Telemetry telemetry; // for FTC dashboard--will integrate later

    // variables for claw positions
    public final double open_pos = 0;
    public final double closed_pos = 0.45;

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

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // setting the mode of each motor to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //start at 0 power
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
         * */
        frontLeft.setPower(Range.clip(frontLeftPower, -MAX_POWER, MAX_POWER));
        frontRight.setPower(Range.clip(frontRightPower, -MAX_POWER, MAX_POWER));
        backLeft.setPower(Range.clip(backLeftPower, -MAX_POWER, MAX_POWER));
        backRight.setPower(Range.clip(backRightPower, -MAX_POWER, MAX_POWER));
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

        final double TICKS_PER_REV = 537.7; // encoder resolution
        final double WHEEL_DIAMETER = 104 / 25.4; // inches
        final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAMETER*Math.PI);

        double target = inches * TICKS_PER_IN;; // may have to find a way to convert from inches to ticks

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
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    public void turnDegrees(double targetDegrees, String dir) {
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
        final double WHEEL_DIAMETER = 104 / 25.4; // inches
        final double TICKS_PER_IN = TICKS_PER_REV / (WHEEL_DIAMETER*Math.PI);

        int targetPosition = (int) (target_inches * TICKS_PER_IN);

        double leftPowerLevel = MAX_POWER;
        double rightPowerLevel = MAX_POWER;
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

//
//        // Wait until the motor reaches the target position
       while (frontLeft.isBusy()) {
//            telemetry.addData("Left Front Current Position", frontLeft.getCurrentPosition());
//            telemetry.addData("Left Back Current Position", backLeft.getCurrentPosition());
//            telemetry.addData("Right Front Current Position", frontRight.getCurrentPosition());
//            telemetry.addData("Right Back Current Position", backRight.getCurrentPosition());
            telemetry.update();
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
        }
        else{
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int pos = slide.getCurrentPosition();
            slide.setTargetPosition(pos);
            slide.setPower(0.001);
        }
    }
    // rotates pulley a certain number of rotations to lift the slide
    // two cases: up and down
    public void liftSlideAuto(double inches, String dir){
        double mm = inches * 25.4; // converting inches to mm
        final double TICKS_PER_REV = 2786.2; // ticks per revolution
        final double MM_PER_ROTATION = 120; // for every 1 rotation, the belt moves 120 MM

        int target = (int)(((2786.2)/(38.2*Math.PI)) * mm); //

        double power = MAX_POWER;
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (dir.equals("down")){
            //go down target ticks
            //neg power neg target
            power *= -1;
            target *= -1;
        }
        slide.setTargetPosition(target);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setPower(power);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    // accesser methods--for debugging purposes
}

