package org.firstinspires.ftc.teamcode;// Use for teleop

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * Saketh Ayyagari
 * Regular Teleop without IMU Assist
 */


@TeleOp(name="RobotRelativeTeleop", group="Iterative OpMode")
//@Disabled
public class RobotRelativeTeleop extends OpMode
{
    // Standard member variables
    private ElapsedTime runtime = new ElapsedTime();
    private final double MAX_POWER = 0.6;
    // robot classes/components
    private Robot drivetrain = new Robot(MAX_POWER);
    private IMU.Parameters myIMUParameters;
    private IMU imu;
    // camera variables: 480p resolution
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private int cameraMonitorViewId;
    private OpenCvWebcam camera;
    private testPipeline pipeline = new testPipeline();
    // PID Values for camera lock
    private final double Kp = 0.003125;
    private final double Kd = 0.0;
    private Double prevError = 0.0;
    private double setpoint_angle;
    private int setpoint_slide;

    private final PIDController slide_control = new PIDController(0.075);
    private final PIDController angle_lock = new PIDController(0.075, true);
    private void initCamera(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam13115");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //start streaming from here
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error. Please try again!", null);
            }
        });
    }
    @Override
    public void init() {
        initCamera();
        drivetrain.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        // initializing IMU with parameters
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        //reset yaw
        imu.resetYaw();
        setpoint_angle = imu.getRobotYawPitchRollAngles().getYaw();
        // initialize arm to be in upper position
        drivetrain.liftServo(0.35);
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // gets joystick values for translational motion (drive and strafe) and turning
        double drive = -gamepad1.left_stick_y; // moving forward or backward
        double turn = gamepad1.right_stick_x; // strafing left or right
        double strafe = gamepad1.left_stick_x; // turning clockwise or counterclockwise
        double lift = gamepad1.left_trigger - gamepad1.right_trigger; // lifting the slide
        String state = "unlock";
        // IMU-Assist for Teleop
        // changes state variable based on controller input
        if (Math.abs(turn) < 0.01){
            state = "unlock";
        }
        else{
            state = "lock";
        }
        switch(state){
            case "lock": // turn value is PD output
                turn = angle_lock.update(setpoint_angle, imu.getRobotYawPitchRollAngles().getYaw());
                break;
            case "unlock": // non-zero turn value changes setpoint
                setpoint_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                break;
        }
        // Closed-loop control for slide
        // if the power of the slide motor is within a small range, have the power sent be
        // closed loop control
        if (Math.abs(lift) < 0.001){
            int setpoint = drivetrain.slide.getCurrentPosition();
            lift = slide_control.update(setpoint,
                    drivetrain.slide.getCurrentPosition()
            );
            telemetry.addData("Slide Setpoint: ", setpoint);
            telemetry.addData("Slide Current Value: ", drivetrain.slide.getCurrentPosition());
            telemetry.addData("Slide Power sent (NEGATIVE MUST BE UP): ", lift);
        }

        telemetry.addData("drive: ", drive);
        telemetry.addData("turn: ", turn);
        telemetry.addData("strafe: ", strafe);
        telemetry.addLine();
        // gamepad button controls
        if (gamepad1.left_bumper){
            drivetrain.close();
        }
        else if (gamepad1.right_bumper){
            drivetrain.open();
        }
        if (gamepad1.a){
            drivetrain.liftServo(0.01);
        }
        if (gamepad1.b){
            drivetrain.liftServo(0.35);
        }
        // second controller controls: semi-autonomous
        if (gamepad2.a){
            drivetrain.liftServo(0);
            sleep(300);
            drivetrain.close();
            sleep(300);
            drivetrain.liftServo(0.3);
        }
        // camera tracking
        if (gamepad1.y){
            Point center = pipeline.get_contour_center();
            int actual = (int)center.x;
            final double SETPOINT = ((CAMERA_WIDTH)/2) + 45;
            strafe = PIDControl(SETPOINT, actual);
        }
        if(gamepad1.dpad_left){
            pipeline.track_blue();
            telemetry.addData("Tracked Color", "Blue");
        }
        if(gamepad1.dpad_right){
            pipeline.track_red();
            telemetry.addData("Tracked Color", "Red");
        }
        if(gamepad1.dpad_up){
            pipeline.track_yellow();
            telemetry.addData("Tracked Color", "Yellow");
        }
        telemetry.addData("Contour Center: ", pipeline.get_contour_center());
        // field-relative driving instead of robot-relative driving

         // gets heading in radians
         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
         double angle = orientation.getYaw(AngleUnit.RADIANS);

        drivetrain.powerChassisMotors(drive, turn, strafe);
        drivetrain.liftSlide(lift);

        telemetry.addData("frontLeft Power: ", drivetrain.frontLeft.getPower());
        telemetry.addData("backLeft Power: ", drivetrain.backLeft.getPower());
        telemetry.addData("frontRight Power: ", drivetrain.frontRight.getPower());
        telemetry.addData("backRight Power: ", drivetrain.backRight.getPower());
        telemetry.addLine();
        telemetry.addData("Extend Pos: ", drivetrain.extend.getPosition());
        telemetry.addData("Left Pos: ", drivetrain.left.getPosition());
        telemetry.addData("Right Pos: ", drivetrain.right.getPosition());
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("IMU Angle", angle);
        telemetry.update();
    }
    public double PIDControl(double setpoint, double current){
        double error = setpoint - current;
        double P_error = Kp*error;
        // calculates derivative error
        double D_error = Kd * (error - prevError)/runtime.seconds();
        prevError = error;
        // resets timer for recalculating derivative error
        resetRuntime();

        return Range.clip(P_error + D_error, -1, 1);
    }
}
