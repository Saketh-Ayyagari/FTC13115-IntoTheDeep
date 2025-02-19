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
         * **/
        if (opModeIsActive()){
            int milli = 750;
            robot.close();
            sleep(milli);
            robot.moveRobotwEncoders("forward", 25, robot.MAX_POWER);
            sleep(milli);
            robot.liftSlide(9, "up");
            sleep(milli);
            robot.moveRobotwEncoders("forward", 1, 0.1);
            sleep(milli);
            robot.liftSlide(6, "down");
            sleep(milli);
            robot.open();
            sleep(milli);
            robot.moveRobotwEncoders("backward", 24, robot.MAX_POWER);
        }
    }
}
