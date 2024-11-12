package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="IMUAutonomousRelativeTurning", group="Autonomous")
public class IMUAutonomousRelativeTurning extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot drivetrain;
    private IMU.Parameters myIMUParameters;
    private IMU imu;

    private static final double MAX_POWER = 0.5;

    @Override
    public void runOpMode() {
        // Initial setup for IMU
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drivetrain = new Robot(MAX_POWER);
        drivetrain.init(hardwareMap);

        // Initializing the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        imu.resetYaw(); // Reset the IMU heading to 0 degrees

        waitForStart();
        runtime.reset();

        // Example action: Turn 90 degrees clockwise and move forward
        turnRelative(90);  // Turn 90 degrees clockwise
        drivetrain.moveRobotwEncoders("forward", 10);

        turnRelative(90);  // Turn another 90 degrees (total 180 degrees from start)
        drivetrain.moveRobotwEncoders("forward", 10);
    }

    /**
     * Turn the robot to a relative target heading.
     * @param relativeDegrees The degrees to turn relative to the current heading.
     */
    public void turnRelative(double relativeDegrees) { // must be between -180 and 180
        // Create an object to receive the IMU angles
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // reset the yaw
        imu.resetYaw();

        // Keep turning until the robot heading is greater than the target heading
        while (orientation.getYaw() < relativeDegrees) {
            // Simple control: turn left or right based on the heading difference
            if (getHeading() > 0) {
                drivetrain.powerMotors(0, MAX_POWER, 0); // Turn right
            } else {
                drivetrain.powerMotors(0, -MAX_POWER, 0); // Turn left
            }

            telemetry.addData("Current Heading", orientation.getYaw());
            telemetry.addData("Target Heading", orientation.getYaw());
            telemetry.update();
        }
        // Stop the robot once the target heading is reached
        drivetrain.powerMotors(0, 0, 0); // Stop motors
    }

    /**
     * Get the current heading of the robot in degrees.
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}