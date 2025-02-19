package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

// Detecting contours of specific colors
public class testPipeline extends OpenCvPipeline {
    private Mat output = new Mat();

    // NOTE: "Scalar" color channels are RGB
    private Scalar low_hsv_RED = new Scalar(0, 125, 75);
    private Scalar high_hsv_RED = new Scalar(10, 255, 255);

    private Scalar low_hsv_BLUE = new Scalar(100, 125, 75);
    private Scalar high_hsv_BLUE = new Scalar(115, 255, 255);

    private Scalar low_hsv_YELLOW = new Scalar(23, 150, 75);
    private Scalar high_hsv_YELLOW = new Scalar(34, 255, 255);



    private Scalar low_hsv = low_hsv_YELLOW;
    private Scalar high_hsv = high_hsv_YELLOW;
    private final Scalar CONTOUR_COLOR = new Scalar(0, 0, 125);
    // Mat variables used for find_contours() method
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat(); // NOT necessary for most tasks

    // variables for contours--added to telemetry
    private Point contour_center = new Point();
    private double contour_area = 0;

    @Override
    public Mat processFrame(Mat input) {
        // finds contours
        ArrayList<MatOfPoint> contour_list = find_contours(input, low_hsv, high_hsv, 100);
        input.copyTo(output);

        // finding max contour
        MatOfPoint max_contour = get_largest_contour(contour_list);
        if (max_contour != null){
            // gets center and area for telemetry variables
            this.contour_center = find_contour_center(max_contour);
            this.contour_area = Imgproc.contourArea(max_contour);

            // draws contours
            ArrayList<MatOfPoint> contour = new ArrayList<>();
            contour.add(max_contour);
            Imgproc.drawContours(output, contour, -1, CONTOUR_COLOR, 8);
            // Draw a dot at the centroid
//            String label = "(" + (int) contour_center.x + ", " + (int) contour_center.y + ")";
//            Imgproc.putText(input, label, new Point(contour_center.x - 10, contour_center.y),
//                    Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255), 6);
//
            Imgproc.circle(output, contour_center, 8, new Scalar(0, 255, 0), -1);
        }

        return output;
    }

    // gets a list of contours
    public ArrayList<MatOfPoint> find_contours(Mat image, Scalar low, Scalar high, double min_contour_area) {

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_RGB2HSV); // converts RGB image to HSV
        Core.inRange(hsv, low, high, mask); // creating mask
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // only including contours with an area greater than specified
        // (gets rid of unnecessary tiny "specks")
        ArrayList<MatOfPoint> filtered_contours = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= min_contour_area) {
                filtered_contours.add(contour);
            }
        }
        return filtered_contours;
    }
    // finds the largest contour in a list
    // returns null
    public MatOfPoint get_largest_contour(ArrayList<MatOfPoint> contour_list){
        MatOfPoint max_contour = null;
        double max_contour_area = -1;
        for (MatOfPoint contour : contour_list) {
            if (Imgproc.contourArea(contour) > max_contour_area) {
                max_contour = contour;
                max_contour_area = Imgproc.contourArea(contour);
            }
        }
        return max_contour;
    }
    // gets the pixel of the contour center
    // y and x are reversed to match standard indexing
    public Point find_contour_center(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        if (moments.get_m00() != 0) {
            int cx = (int) (moments.get_m10() / moments.get_m00());
            int cy = (int) (moments.get_m01() / moments.get_m00());
            return new Point(cx, cy);
        } else {
            return new Point(0, 0);
        }
    }
    public Point get_contour_center(){
        return this.contour_center;
    }
    public double get_contour_area(){
        return this.contour_area;
    }

    public void track_blue(){
        low_hsv = low_hsv_BLUE;
        high_hsv = high_hsv_BLUE;
    }
    public void track_red(){
        low_hsv = low_hsv_RED;
        high_hsv = high_hsv_RED;
    }

    public void track_yellow(){
        low_hsv = low_hsv_YELLOW;
        high_hsv = high_hsv_YELLOW;
    }
}