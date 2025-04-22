package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class MotorTest extends LinearOpMode {
    private final Robot robot = new Robot(0.8);
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.liftServo(0.35);
        runtime.reset();
        if (opModeIsActive()){
            robot.liftSlide(27, "up");
            sleep(500);
            robot.liftSlide(12, "down");
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