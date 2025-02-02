// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


@TeleOp(name="TrackingTeleOp", group="Iterative OpMode")
//@Disabled
public class TrackingTeleOp extends OpMode {


    private final double MAX_POWER = 0.6;
    private double power = 0;
    private final double open_pos = 0.45;
    private final double closed_pos = 0.88;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot enma = new Robot(MAX_POWER); // one piece reference lol

    private IMU.Parameters myIMUParameters;
    private IMU imu;

    // PID Values
    private final double Kp = 0.003125;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private Double prevError = 0.0;
    private double error_sum = 0;

    // camera variables
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private int cameraMonitorViewId;
    private OpenCvWebcam camera;
    private WebcamName webcamName;
    private testPipeline pipeline = new testPipeline();

    private void initCamera(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "webcam13115");
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
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Tracked Color", "Red");
        telemetry.update();

        enma.init(hardwareMap);
        initCamera();

        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUParameters);
        imu.resetYaw();
    }
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // gets heading in radians
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = orientation.getYaw(AngleUnit.RADIANS);
        // gets joystick values for translational motion (drive and strafe) and rotational
        // motion
        double drive = -gamepad1.left_stick_y; // moving forward or backward
        double turn = -gamepad1.right_stick_x; // strafing left or right
        double strafe = -gamepad1.left_stick_x; // turning clockwise or counterclockwise
        double lift = gamepad1.left_trigger - gamepad1.right_trigger; // lifting the slide

        // field-relative driving instead of robot-relative driving

//        double x_rotated = drive * Math.cos(angle) - strafe * Math.sin(angle);
//        double y_rotated = drive * Math.sin(angle) + strafe * Math.cos(angle);
//        drivetrain.powerChassisMotors(x_rotated, turn, y_rotated); // sends individual powers to the motors

        // robot-relative driving settings--COMMENT ABOVE 3 LINES AND COMMENT OUT THESE LINES FOR
        //   TESTING!!
        enma.powerChassisMotors(drive, turn, strafe);


        //TESTING WITHOUT INTAKE ATTACHED UNCOMMENT THESE ONCE IT IS ATTACHED!!!!!
        //enma.liftSlide(lift);

        /*if (gamepad1.left_bumper){
            enma.close();
        }
        else if (gamepad1.right_bumper){
            enma.open();
        }
        if (gamepad1.x){
            enma.liftServo(0.4);
        }
        if (gamepad1.a){
            enma.liftServo(0.5);
        }
        if (gamepad1.b){
            enma.liftServo(0.85);
        }*/
        if (gamepad1.y){
            Point center = pipeline.get_contour_center();
            int actual = (int)center.x;
            double SETPOINT = (CAMERA_WIDTH)/2;
            double speed = PIDControl(SETPOINT, actual);

            enma.powerChassisMotors(0, speed, 0);
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
