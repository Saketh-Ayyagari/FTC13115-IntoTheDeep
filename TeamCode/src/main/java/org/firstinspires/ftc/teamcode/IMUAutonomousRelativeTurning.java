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
    // initializing robot, IMU, and IMU parameter objects
    private ElapsedTime runtime = new ElapsedTime();
    private Robot drivetrain;
    private IMU.Parameters myIMUParameters;
    private IMU imu;
    // max speed the robot will move
    private static final double MAX_POWER = 0.3;

    @Override
    public void runOpMode() {
        // Initial setup for IMU--initializes parameters based on IMU orientation
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // creating and initializing Robot object
        drivetrain = new Robot(MAX_POWER);
        drivetrain.init(hardwareMap);

        // Initializing the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        imu.resetYaw(); // Reset the IMU heading to 0 degrees

        waitForStart();
        runtime.reset();

        // Example action: Turn 90 degrees clockwise and move forward
        turnRelative(90);  // Turn 90 degrees counterclockwise
        // positive = counterclockwise
        // negative = clockwise
    }

    /**
     * Turn the robot to a target heading relative to the robot.
     * @param relativeDegrees The degrees to turn relative to the current heading.
     */
    public void turnRelative(double relativeDegrees) { // must be between -180 and 180
        imu.resetYaw();
        // Keep turning until the robot heading is greater than the target heading
        while (Math.abs(getHeading()) < Math.abs(relativeDegrees)) {
            // Turn left or right based on the sign of the heading value
            if (relativeDegrees > 0) {
                drivetrain.powerMotors(0, MAX_POWER, 0); // Turn right
            }
            else{
                drivetrain.powerMotors(0, -MAX_POWER, 0);
            }

            telemetry.addData("Current Heading", getHeading());
            telemetry.addData("Target Heading", relativeDegrees);
            telemetry.update();
        }
        // Stop the robot once the target heading is reached
        drivetrain.brake();
    }

    /**
     * Get the current heading of the robot in degrees.
     */
    public double getHeading() { //
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}