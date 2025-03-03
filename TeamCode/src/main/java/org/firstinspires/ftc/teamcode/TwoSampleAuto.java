// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TwoSampleAuto", group="Linear OpMode")
//@Disabled
public class TwoSampleAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot(0.5);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);


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
            robot.moveRobotwEncoders("forward", 31, 0.5);
            robot.liftSlide(8, "down");
            robot.open();
            robot.moveRobotwEncoders("backward", 24, 0.5);
            // strafe left until camera detects sample
        }
    }
}
