// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutonomousV2", group="Linear OpMode")
//@Disabled
public class AutonomousV2 extends LinearOpMode {

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

        //Armless Autonomous Strategy show in Armless Auto Strat 1 Corrected.pdf https://drive.google.com/drive/u/1/folders/1flgkWB1yZEToVUaXiTsMUBPjUwb2EzVf
        //start robot facing the outside on second tile line
        robot.moveRobotwEncodersPID("right", 29, robot.MAX_POWER);
        //robot.turnDegrees("90", );

        //push first sample
        robot.moveRobotwEncodersPID("backward", 58, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("right", 9, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("forward", 56, robot.MAX_POWER);

        //push second sample
        robot.moveRobotwEncodersPID("backward", 54, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("right", 14, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("forward", 56, robot.MAX_POWER);

        //push third sample
        robot.moveRobotwEncodersPID("backward", 52, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("right", 12, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("forward", 56, robot.MAX_POWER);

        //park in ascent zone
        robot.moveRobotwEncodersPID("backward", 58, robot.MAX_POWER);
        robot.moveRobotwEncodersPID("left", 40, robot.MAX_POWER);

    }
}
