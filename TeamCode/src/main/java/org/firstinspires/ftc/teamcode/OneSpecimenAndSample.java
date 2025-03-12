// for autonomous

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="OneSpecimenAndSample", group="Linear OpMode")
@Disabled
public class OneSpecimenAndSample extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot(0.5);

    /*
    Variables for camera initialization
    camera variables: 480p resolution
    */
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private int cameraMonitorViewId;
    private OpenCvWebcam camera;
    private testPipeline pipeline = new testPipeline();

    private final PIDController sample_lock = new PIDController(0.003125);

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
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // initializing hardware
        robot.init(hardwareMap);
        initCamera();
        // closing on specimen and going to initial extension position
        robot.close();
        robot.liftServo(0.34);
        // Wait for the game to start (driver presses START)
        waitForStart();

        runtime.reset();
        /** Autonomous Strategy:
         * Hang one specimen onto high bar
         * Park
         **/
        if (opModeIsActive()){
            robot.liftSlide(17.5, "up");

            robot.moveRobotwEncoders("forward", 29, 0.5);
            robot.liftSlide(6, "down");
            robot.open();
            robot.moveRobotwEncoders("backward", 24, 0.5);
            robot.moveRobotwEncoders("left", 24, 0.5);
            // aligning with sample
            Point center = pipeline.get_contour_center();
            int actual = (int)center.x;
            final double SETPOINT = (CAMERA_WIDTH)/2;
            double strafe = sample_lock.update(SETPOINT, actual);

            robot.powerChassisMotors(0, strafe, 0);
        }
    }
}
