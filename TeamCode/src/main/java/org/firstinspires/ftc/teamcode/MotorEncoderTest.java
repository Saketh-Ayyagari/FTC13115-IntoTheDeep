package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
}