// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Robot1_TestSuite", group="Linear OpMode")
//@Disabled
public class Robot1_TestSuite extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot enma = new Robot(0.8); // one piece reference lol

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        enma.init(hardwareMap);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();


        enma.moveRobotwEncoders("forward", 10);
        enma.turnDegrees(90, "clockwise");
        enma.moveRobotwEncoders("forward", 10);// run once
        enma.turnDegrees(90, "clockwise");
        enma.moveRobotwEncoders("forward", 10);
        enma.turnDegrees(90, "clockwise");
        enma.moveRobotwEncoders("forward", 10);// run once

        enma.turnDegrees(90, "clockwise");

        enma.turnDegrees(90, "counterclockwise");
        enma.liftSlideAuto(100, "up");
        enma.liftSlideAuto(100, "down");

        if (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
