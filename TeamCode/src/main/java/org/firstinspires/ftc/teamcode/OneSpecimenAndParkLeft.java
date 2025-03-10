// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Places one specimen and parks in the observation zone
 * USE IF  YOU'RE STARTING ON THE LEFT
 * **/
@Autonomous(name="OneSpecimenAndPark", group="Linear OpMode")
//@Disabled
public class OneSpecimenAndParkLeft extends LinearOpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot(0.5);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // initializing hardware
        robot.init(hardwareMap);
        // closing on specimen and going to initial extension position
        robot.close();
        robot.liftServo(0.35);
        // Wait for the game to start (driver presses START)
        waitForStart();

        runtime.reset();
        /** Autonomous Strategy:
         * Hang one specimen onto high bar
         * Park
         **/
        if (opModeIsActive()){
            robot.liftSlide(17.5, "up");

            robot.moveRobotwEncoders("forward", 30, 0.5);
            robot.liftSlide(7, "down");
            robot.open();
            robot.moveRobotwEncoders("backward", 24, 0.5);
            robot.moveRobotwEncoders("right", 50, 0.5);
        }
    }
}
