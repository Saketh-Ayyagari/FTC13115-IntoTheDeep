package org.firstinspires.ftc.teamcode;// Use for teleop

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * Field-Relative Teleop. USE ONLY IF IMU IS INTIALIZED TO 0 RAD VERTICALLY TO FIELD
 */
@TeleOp(name="FieldRelativeTeleOp", group="Iterative OpMode")
@Disabled
public class FieldRelativeTeleop extends OpMode
{
    // Standard member variables
    private ElapsedTime runtime = new ElapsedTime();
    private final double MAX_POWER = 0.7;
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
    // PID Values
    private final double Kp = 0.003125;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private Double prevError = 0.0;
    private double error_sum = 0;


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
        // initialize arm to be in upper position
        drivetrain.liftServo(0.33);
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
            drivetrain.liftServo(0);
        }
        if (gamepad1.b){
            drivetrain.liftServo(0.33);
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
         double drive_rotated = drive * Math.cos(angle) - strafe * Math.sin(angle);
        double strafe_rotated = drive * Math.sin(angle) + strafe * Math.cos(angle);
        drivetrain.powerChassisMotors(drive_rotated, turn, strafe_rotated); // sends individual powers to the motors

        // robot-relative driving settings--COMMENT ABOVE 3 LINES AND COMMENT OUT THESE LINES FOR
//        drivetrain.powerChassisMotors(drive, turn, strafe);
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

        return Range.clip(P_error + D_error, -0.3, 0.3);
    }
}
