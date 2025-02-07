package org.firstinspires.ftc.teamcode;// Use for teleop

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * Saketh Ayyagari
 * Regular Teleop without IMU Assist
 */


@TeleOp(name="Teleop", group="Iterative OpMode")
//@Disabled
public class NoIMUTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private final double MAX_POWER = 0.3;
    private double power = 0;

    // robot bits
    private Robot drivetrain = new Robot(MAX_POWER);
    private IMU.Parameters myIMUParameters;
    private IMU imu;
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        // initializing IMU with parameters
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        //reset yaw
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // gets heading in radians
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = orientation.getYaw(AngleUnit.RADIANS);
        // gets joystick values for translational motion (drive and strafe) and rotational
        // motion
        double drive = -gamepad1.left_stick_y; // moving forward or backward
        double turn = gamepad1.right_stick_x; // strafing left or right
        double strafe = -gamepad1.left_stick_x; // turning clockwise or counterclockwise
        double lift = gamepad1.left_trigger - gamepad1.right_trigger; // lifting the slide

        telemetry.addData("drive: ", drive);
        telemetry.addData("turn: ", turn);
        telemetry.addData("strafe: ", strafe);
        telemetry.addLine();
        // field-relative driving instead of robot-relative driving

//        double x_rotated = drive * Math.cos(angle) - strafe * Math.sin(angle);
//        double y_rotated = drive * Math.sin(angle) + strafe * Math.cos(angle);
//        drivetrain.powerChassisMotors(x_rotated, turn, y_rotated); // sends individual powers to the motors

        // robot-relative driving settings--COMMENT ABOVE 3 LINES AND COMMENT OUT THESE LINES FOR
        //   TESTING!!
        drivetrain.powerChassisMotors(drive, turn, strafe);
        drivetrain.liftSlide(lift);

        if (gamepad1.left_bumper){
            drivetrain.close();
        }
        else if (gamepad1.right_bumper){
            drivetrain.open();
        }
        if (gamepad1.x){
            drivetrain.liftServo(-0.2);
        }
        if (gamepad1.a){
            drivetrain.liftServo(0);
        }
        if (gamepad1.b){
            drivetrain.liftServo(0.33);
        }
        telemetry.addData("frontLeft Power: ", drivetrain.frontLeft.getPower());
        telemetry.addData("backLeft Power: ", drivetrain.backLeft.getPower());
        telemetry.addData("frontRight Power: ", drivetrain.frontRight.getPower());
        telemetry.addData("backRight Power: ", drivetrain.backRight.getPower());
        telemetry.addLine();
        telemetry.addData("Extend Pos: ", drivetrain.extend.getPosition());
        telemetry.addData("Left Pos: ", drivetrain.left.getPosition());
        telemetry.addData("Right Pos: ", drivetrain.right.getPosition());
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();
    }
}
