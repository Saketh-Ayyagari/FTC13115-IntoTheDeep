// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="TwoSampleAuto", group="Linear OpMode")
//@Disabled
public class TwoSampleAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot(0.5);
    private Rev2mDistanceSensor sensor_right;
    private ColorRangeSensor sensor_left;

    private PIDController move = new PIDController(0.03125);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        sensor_right = hardwareMap.get(Rev2mDistanceSensor.class, "distance-right");
        sensor_left = hardwareMap.get(ColorRangeSensor.class, "distance-left");

        // Wait for the game to start (driver presses START)
        waitForStart();

        runtime.reset();
        /** Autonomous Strategy:
         * Hang one specimen onto high bar
         * Place two samples in low basket (or high basket if possible)
         **/
        if (opModeIsActive()){
            robot.close();
            robot.liftSlide(17.5, "up");
            double distance_left = sensor_left.getDistance(DistanceUnit.INCH);
            double distance_right = sensor_right.getDistance(DistanceUnit.INCH);

            String state = "ALIGN";
            final double tolerance = 0.4;
            final int target = 4; //target distance in inches

            while (!state.equals("DONE")){
                distance_left = sensor_left.getDistance(DistanceUnit.INCH);
                distance_right = sensor_right.getDistance(DistanceUnit.INCH);

                if (Math.abs(distance_left - distance_right) > tolerance){
                    state = "ALIGN";
                }else{
                    state = "FORWARD";
                }
                if (Math.abs((distance_left + distance_right)/2) < target){
                    state = "DONE";
                }

                if (state.equals("ALIGN")){
                    if (distance_left < distance_right){//we need to align by turning clockwise so send positive turning power
                        robot.powerChassisMotors(0, robot.MAX_POWER/4, 0);
                    }else {//need to align and turn counterclockwise so send negative turn power
                        robot.powerChassisMotors(0, -robot.MAX_POWER/4, 0);
                    }
                }else if (state.equals("FORWARD")){
                    robot.powerChassisMotors(robot.MAX_POWER/2, 0, 0);
                }
                telemetry.addData("distanceleft", distance_left);
            }

            robot.brake();

            //robot.moveRobotwEncoders("forward", 31, 0.5);
            robot.liftSlide(8, "down");
            robot.open();
            robot.moveRobotwEncoders("backward", 24, 0.5);
            // strafe left until camera detects sample
        }
    }
}
