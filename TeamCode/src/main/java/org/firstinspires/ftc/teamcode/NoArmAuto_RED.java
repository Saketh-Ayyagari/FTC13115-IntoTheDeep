////Autonomous Code
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
//
//@Autonomous(name="NoArmAuto_RED", group="Linear Opmode")
//@Disabled
//public class NoArmAuto_RED extends LinearOpMode{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private Robot drivetrain = new Robot(0.8);
//    Servo left, right;
//    private IMU.parameters parameters;
//    private IMU imu;
//    @Override
//    public void runOpMode(){
//        // defining IMU parameters
//         parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
//        );
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        drivetrain = new Robot(0.8);
//        drivetrain.init(hardwareMap);
//        // initializing IMU
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(parameters);
//        imu.resetYaw();
//
//        waitForStart();
//        runtime.reset();
//        if (opModeIsActive()){ //actual autonomous
//
//            // TILE SIZE: 24 inches per square. Assume start is in the center of the tile
//            // 1 inch = 25.4 mm
//            // Direction options: forward, backward, counterclockwise, clockwise,
//            // strafe_left, strafe_right
//            drivetrain.moveRobotwEncoders("forward", 24);
//            sleep(250);
//            drivetrain.moveRobotwEncoders("counterclockwise",  ); // figure out how many inches for 90 degree turn
//            sleep(250);
//            drivetrain.moveRobotwEncoders("forward", 6);
//            sleep(250);
//            drivetrain.moveRobotwEncoders(); // strafe left ~ 20 inches
//            sleep(250);
//            drivetrain.moveRobotwEncoders("forward", 20);
//            sleep(250)
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//
//            telemetry.update();
//        }
//
//    }
//
//}