package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedWarehouseTeamElementPipeline extends OpenCvPipeline {

    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MID,
        RIGHT
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(85, 120), //Formally 35
            new Point(125, 160)); //Formally 75

    static final Rect RIGHT_ROI = new Rect(
            new Point(235, 120), //Formally 35
            new Point(273, 160)); //Formally 75
    static double PERCENT_COLOR_THRESHOLD = 0.3;


    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(50, 0, 0); //HEX #579884
        Scalar highHSV = new Scalar(80, 255, 255); //HEX #9BDABF

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);
        Mat left = mat.submat(LEFT_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255; //Formally LEFT_ROI.area() / 255
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255; //Formally RIGHT_ROI.area() / 255

        left.release();
        right.release();

        boolean elementMID = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;

        if (elementLeft) {
            location = Location.LEFT;
            //telemetry.addData("Element Location: ", "LEFT");
        }

        else if (elementMID){
            location = Location.MID;
            //telemetry.addData("Element Location: ", "MID");
        }

        else {
            location = Location.RIGHT;
            //telemetry.addData("Element Location: ", "RIGHT");
        }
        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);


        Scalar leftField = new Scalar(255, 0, 0);
        Scalar rightField = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.LEFT? rightField:leftField);
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.RIGHT? rightField:leftField);




        return mat;
    }

    public RedWarehouseTeamElementPipeline.Location getLocation(){
        return location;
    }


}
