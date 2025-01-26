package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
/**
 * Saketh Ayyagari
 * Test Suite for different motors. Comment anything needed for customization
 * **/
@TeleOp(name="MotorTest", group="Linear OpMode")
public class MotorTest extends LinearOpMode {
    private DcMotor test;
    private Servo claw;
    private ElapsedTime runtime = new ElapsedTime();
    private final double open_pos = 0.45;
    private final double closed_pos = 0.88;
    @Override
    public void runOpMode() {
        test = hardwareMap.get(DcMotor.class, "test");
        claw = hardwareMap.get(Servo.class, "claw");
        double position = open_pos;
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            double power = gamepad1.left_stick_y;
            test.setPower(power);
            if (gamepad1.a){
                position = closed_pos;
            }
            else if (gamepad1.b){
                position = open_pos;
            }
            claw.setPosition(position);
            telemetry.addData("Servo Position", claw.getPosition());
            telemetry.update();
        }
    }
    public double clamp(double x, double low, double high){
        if (x < low){
            return low;
        }
        else if (x > high){
            return high;
        }
        return x;
    }
}