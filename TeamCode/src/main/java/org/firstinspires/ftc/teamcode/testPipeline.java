package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class testPipeline extends OpenCvPipeline {
    Mat submat;

//    @Override
//    public void init(Mat firstFrame){
//        submat = firstFrame.submat(0,640,0,480);
//    }
    @Override
    public Mat processFrame(Mat input){
        submat = input;
        // do stuff here
        return submat;
    }

}
