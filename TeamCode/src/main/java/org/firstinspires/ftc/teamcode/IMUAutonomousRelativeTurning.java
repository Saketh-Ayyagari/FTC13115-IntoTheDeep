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

    // PID Values
    private final double Kp = -0.03125;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private Double prevError = 0.0;
    private double error_sum = 0;

    // Tolerance for PID control and setpoint change
    private double tolerance = 0.5; // degree tolerance to consider heading reached
    private static final double MAX_POWER = 0.4;

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


        drivetrain = new Robot(0.5);
        drivetrain.init(hardwareMap);

        // Initializing the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        imu.resetYaw();

        waitForStart();
        runtime.reset();

        // Example action: Turn 90 degrees clockwise and move forward
        turnToHeading(90);   // Turn 90 degrees
        drivetrain.moveRobotwEncoders("forward", 10, drivetrain.MAX_POWER );
        turnToHeading(180);  // Turn to 180 degrees
        drivetrain.moveRobotwEncoders("forward", 10, drivetrain.MAX_POWER );
    }

    /**
     * Turn the robot to the specified heading.
     * @param relativeTargetHeading The heading in degrees to turn to.
     */
    public void turnToHeading(double relativeTargetHeading) {
        double setpoint = angleWrap(getHeading() + relativeTargetHeading);  // Calculate new heading setpoint
        double currentHeading = getHeading();

        // Turn until the setpoint is reached
        while (Math.abs(angleWrap(setpoint - currentHeading)) > tolerance) {
            currentHeading = getHeading();
            double turn = PIDControl(setpoint, currentHeading);
            drivetrain.powerMotors(0, turn, 0); // only turning
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Target Setpoint", setpoint);
            telemetry.update();
        }

    }

    /**
     * Move the robot forward for the specified duration.
     * @param moveDuration The time in seconds to move forward.
     */

    /**
     * PID controller for maintaining the heading.
     */
    public double PIDControl(double setpoint, double current){
        double error = angleWrap(setpoint - current);
        double P_error = Kp * error;
        double D_error = Kd * (error - prevError) / runtime.seconds();
        prevError = error;
        // Resets timer for recalculating derivative error
        resetRuntime();

        return Range.clip(P_error + D_error, -MAX_POWER, MAX_POWER);
    }

    /**
     * Ensure angle stays between -180 and 180 degrees.
     */
    public double angleWrap(double degrees) {
        if (degrees > 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    /**
     * Get the current heading of the robot in degrees.
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
