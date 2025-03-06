// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TwoSampleAuto", group="Linear OpMode")
//@Disabled
public class TwoSampleAndPark extends LinearOpMode {

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
         * Park
         **/
        if (opModeIsActive()){
            robot.close();
            robot.liftSlide(17.5, "up");

            robot.goToPosition(0, 29, 0.5);
            robot.liftSlide(6, "down");
            robot.open();
            robot.goToPosition(0, 5, 0.5);
            robot.turnDegrees(90, "counterclockwise", 0.5);
            robot.goToPosition(0, 50, 0.5);
            // strafe left until camera detects sample
        }
    }
}
