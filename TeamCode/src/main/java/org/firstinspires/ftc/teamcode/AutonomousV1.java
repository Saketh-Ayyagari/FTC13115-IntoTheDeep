// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Robot1_TestSuite", group="Linear OpMode")
//@Disabled
public class AutonomousV1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot(0.8);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        //Armless Autonomous Strategy show in Armless Auto Strat 1 Corrected.pdf https://drive.google.com/drive/u/1/folders/1flgkWB1yZEToVUaXiTsMUBPjUwb2EzVf
        //start robot facing the outside on second tile line
        robot.moveRobotwEncoders("right", 10, robot.MAX_POWER);

        //push first sample
        robot.moveRobotwEncoders("backwards", 66, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 15, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 64, robot.MAX_POWER);

        //push second sample
        robot.moveRobotwEncoders("backwards", 64, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 24, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 64, robot.MAX_POWER);

        //push third sample
        robot.moveRobotwEncoders("backwards", 64, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 24, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 64, robot.MAX_POWER);

        //park in ascent zone
        robot.moveRobotwEncoders("backwards", 64, robot.MAX_POWER);
        robot.moveRobotwEncoders("left", 60, robot.MAX_POWER);

    }
}
