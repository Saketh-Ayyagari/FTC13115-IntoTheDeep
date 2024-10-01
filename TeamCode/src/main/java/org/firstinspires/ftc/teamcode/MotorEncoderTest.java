package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderMotorTest", group="Linear Opmode")

public class MotorEncoderTest extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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
        moveMillimeters(targetMM, targetMM); //move forward 20cm
        sleep(1000);
        //moveMillimeters(targetMM/2, "backwards"); //back 10
        //moveMillimeters(targetMM/2, "forwards"); //forward 10
        moveMillimeters(-targetMM, -targetMM); //back 20 hopefully to starting position
        sleep(1000);
        moveMillimeters(-targetMM, targetMM); //back 20 hopefully to starting position
        sleep(1000);

        moveMillimeters(targetMM, -targetMM); //back 20 hopefully to starting position
        sleep(1000);

        // moveMillimeters(targetMM, "forwards");
        //turnDegrees(180, "clockwise");
        //sleep(1000);

        //turnDegrees(180, "counterclockwise");
        //sleep(1000);


        //moveMillimeters(targetMM/2, "forwards");
        //turnDegrees(360, "counterclockwise");
        //moveMillimeters(targetMM/2, "forwards");
    }
//
//    public void turnDegrees(double targetDegrees, String dir){
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //need to figure out how to do the math between turning motors and how it relates to the robot actually turning
//        double conversionTickOverMM = 1.624; //originally 1.6243
//        //distance wheels need to travel is the arc length of the circle circumscribed by the base of the robot
//        //(ASSUMING THE ROBOT IS A SQUARE AND EACH WHEEL IS EQUA-DISTANT)
//        //radius is the side length of the base (18 for now)/2 time sqrt2 (makes a 45 45 90 triangle)
//        //then to find arc length its theta (r)
//        //we get degrees so convert to radians and multiply by our radius
//        double baseSize = 18;
//        double radius = (baseSize/2) * (Math.sqrt(2));
//        double radians = (Math.PI*targetDegrees)/180;
//        double targetInch = radius * radians; //arc length = theta r
//        double targetMM = targetInch * 25.4; //convert inch to mm
//
//        int targetPosition = (int)(targetMM * conversionTickOverMM);
//
//        // Set the motor power
//        double leftPowerLevel = 0;
//        double rightPowerLevel = 0;//powerlevel for all motors because we are only moving forward or backward here
//        if (dir.equals("clockwise")) {
//            leftPowerLevel = 0.25;
//            rightPowerLevel = -0.25;
//            leftFrontDrive.setTargetPosition(targetPosition);
//            leftBackDrive.setTargetPosition(targetPosition);
//            rightFrontDrive.setTargetPosition(-targetPosition);
//            rightBackDrive.setTargetPosition(-targetPosition);
//        }else if (dir.equals("counterclockwise")){
//            leftPowerLevel = -0.25;
//            rightPowerLevel = 0.25;
//            leftFrontDrive.setTargetPosition(-targetPosition);
//            leftBackDrive.setTargetPosition(-targetPosition);
//            rightFrontDrive.setTargetPosition(targetPosition);
//            rightBackDrive.setTargetPosition(targetPosition);
//        }
//
//        leftFrontDrive.setPower(leftPowerLevel);
//        leftBackDrive.setPower(leftPowerLevel);
//        rightFrontDrive.setPower(rightPowerLevel);
//        rightBackDrive.setPower(rightPowerLevel);
//
//
//        // Set the motor to run to the target position
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        // Wait until the motor reaches the target position
//        while (opModeIsActive() && leftFrontDrive.isBusy()) {
//            telemetry.addData("Left Front Current Position", leftFrontDrive.getCurrentPosition());
//            telemetry.addData("Left Back Current Position", leftBackDrive.getCurrentPosition());
//            telemetry.addData("Right Front Current Position", rightFrontDrive.getCurrentPosition());
//            telemetry.addData("Right Back Current Position", rightBackDrive.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Stop the motor
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//
//        // Reset the motor mode
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//    }
    public void moveMillimeters(double targetMMLeft, double targetMMRight){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double conversionTickOverMM = 1.624; //1.6243 originally
        int targetPositionLeft = (int)(targetMMLeft * conversionTickOverMM);
        int targetPositionRight = (int)(targetMMRight * conversionTickOverMM);


        // Set the motor power
        double powerLevel = 0; //powerlevel for all motors because we are only moving forward or backward here
        double powerLevelLeft = 0.5;
        double powerLevelRight = 0.5;

        if (targetPositionRight < 0){
            powerLevelRight *= -1;
        }

        if (targetPositionLeft < 0){
            powerLevelLeft *= -1;
        }


        leftFrontDrive.setTargetPosition(targetPositionLeft);
        leftBackDrive.setTargetPosition(targetPositionLeft);
        rightFrontDrive.setTargetPosition(targetPositionRight);
        rightBackDrive.setTargetPosition(targetPositionRight);

        // Set the motor to run to the target position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(powerLevelLeft);
        leftBackDrive.setPower(powerLevelLeft);
        rightFrontDrive.setPower(powerLevelRight);
        rightBackDrive.setPower(powerLevelRight);


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
}
