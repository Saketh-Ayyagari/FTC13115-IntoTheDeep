package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderMotorTest", group="Linear Opmode")

public class MotorEncoderTest extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    static final double     COUNTS_PER_MOTOR_REV    = 2786.2 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_MM   = 104 ;
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    private final double Kp = 0.03125;
    private final double Ki = 0;
    private final double Kd = 0;
    private Double prevError = 0.0;
    private double error_sum = 0;
    double SETPOINT = 0;

    @Override

    public void runOpMode() {
        // Initialize the hardware variables
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set the motor to run using encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double targetMM = 200;
        moveMillimeters(targetMM, "forwards"); //move forward 20cm
        sleep(1000);
        moveMillimeters(targetMM/2, "backwards"); //back 10
        sleep(1000);
        moveMillimeters(targetMM/2, "forwards"); //forward 10
        sleep(1000);
        moveMillimeters(targetMM, "backwards"); //back 20 hopefully to starting position
        moveMillimeters(targetMM, "forwards");
        turnDegrees(180, "clockwise");
        moveMillimeters(targetMM/2, "forwards");
        turnDegrees(360, "counterclockwise");
        moveMillimeters(targetMM/2, "forwards");
    }

    public void turnDegrees(double targetDegrees, String dir){
        //need to figure out how to do the math between turning motors and how it relates to the robot actually turning
        double conversionTickOverMM = 1.6243;
        //distance wheels need to travel is the arc length of the circle circumscribed by the base of the robot
        //(ASSUMING THE ROBOT IS A SQUARE AND EACH WHEEL IS EQUA-DISTANT)
        //radius is the side length of the base (18 for now)/2 time sqrt2 (makes a 45 45 90 triangle)
        //then to find arc length its theta (r)
        //we get degrees so convert to radians and multiply by our radius
        double baseSize = 18;
        double radius = (baseSize/2) * (Math.sqrt(2));
        double radians = (Math.PI*targetDegrees)/180;
        double targetInch = radius * radians; //arc length = theta r
        double targetMM = targetInch * 25.4; //convert inch to mm

        int targetPosition = (int)(targetMM * conversionTickOverMM);

        leftFrontDrive.setTargetPosition(targetPosition);
        leftBackDrive.setTargetPosition(targetPosition);
        rightFrontDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);


        // Set the motor to run to the target position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power
        double leftPowerLevel = 0;
        double rightPowerLevel = 0;//powerlevel for all motors because we are only moving forward or backward here
        if (dir.equals("clockwise")) {
            leftPowerLevel = 0.5;
            rightPowerLevel = -0.5;
        }else if (dir.equals("counterclockwise")){
            leftPowerLevel = -0.5;
            rightPowerLevel = 0.5;
        }
        leftFrontDrive.setPower(leftPowerLevel);
        leftBackDrive.setPower(leftPowerLevel);
        rightFrontDrive.setPower(rightPowerLevel);
        rightBackDrive.setPower(rightPowerLevel);


        // Wait until the motor reaches the target position
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            telemetry.addData("Left Front Current Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Left Back Current Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Right Front Current Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Right Back Current Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveMillimeters(double targetMM, String dir){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int)(targetMM * COUNTS_PER_MM);

        leftFrontDrive.setTargetPosition(targetPosition);
        leftBackDrive.setTargetPosition(targetPosition);
        rightFrontDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);


        // Set the motor to run to the target position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power
        double powerLevel = 0; //powerlevel for all motors because we are only moving forward or backward here
        if (dir.equals("forwards")) {
            powerLevel = DRIVE_SPEED;
        }else if (dir.equals("backwards")){
            powerLevel = -DRIVE_SPEED;
        }
        leftFrontDrive.setPower(powerLevel);
        leftBackDrive.setPower(powerLevel);
        rightFrontDrive.setPower(powerLevel);
        rightBackDrive.setPower(powerLevel);


        // Wait until the motor reaches the target position
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            telemetry.addData("Left Front Current Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Left Back Current Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("Right Front Current Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Right Back Current Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double angleWrap(double degrees){
        if (degrees>180){
            degrees -= 360;
        }else if(degrees < -180){
            degrees += 360;
        }
        return degrees;
    }
    /*

    public double PIDControl(double setpoint, double current){
        double error = setpoint - current;
        double P_error = Kp*error;
        // calculates derivative error
        double D_error = Kd * (error - prevError)/runtime.seconds();
        prevError = error;
        // resets timer for recalculating derivative error
        resetRuntime();

        return Range.clip(P_error + D_error, -MAX_POWER, MAX_POWER);
    }*/
}