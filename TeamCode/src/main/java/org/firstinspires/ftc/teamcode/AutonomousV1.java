// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutonomousV1", group="Linear OpMode")
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

        robot.liftServo(0.85);
        //Armless Autonomous Strategy show in Armless Auto Strat 1 Corrected.pdf https://drive.google.com/drive/u/1/folders/1flgkWB1yZEToVUaXiTsMUBPjUwb2EzVf
        //start robot facing the outside on second tile line
        robot.moveRobotwEncoders("right", 29, robot.MAX_POWER);
        //robot.turnDegrees("90", );

        //push first sample
        robot.liftServo(0.85);
        robot.moveRobotwEncoders("backward", 58, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 9, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 56, robot.MAX_POWER);

        //push second sample
        robot.moveRobotwEncoders("backward", 54, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 14, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 56, robot.MAX_POWER);

        //push third sample
        robot.moveRobotwEncoders("backward", 52, robot.MAX_POWER);
        robot.moveRobotwEncoders("right", 12, robot.MAX_POWER);
        robot.moveRobotwEncoders("forward", 54, robot.MAX_POWER);

        //park in ascent zone
        robot.moveRobotwEncoders("backward", 59, robot.MAX_POWER);
        robot.moveRobotwEncoders("left", 40, robot.MAX_POWER);

    }
}
