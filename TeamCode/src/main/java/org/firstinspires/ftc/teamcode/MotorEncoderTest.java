package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EncoderMotorTest", group="Linear Opmode")

public class MotorEncoderTest extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;


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
        moveMillimeters(targetMM/2, "backwards"); //back 10
        moveMillimeters(targetMM/2, "forwards"); //forward 10
        moveMillimeters(targetMM, "backwards"); //back 20 hopefully to starting position
    }

    public void moveMillimeters(double targetMM, String dir){
        double conversionTickOverMM = 1.6243;
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
        double powerLevel = 0; //powerlevel for all motors because we are only moving forward or backward here
        if (dir.equals("forwards")) {
            powerLevel = 0.5;
        }else if (dir.equals("backwards")){
            powerLevel = -0.5;
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
