package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class TeamElementPipeline extends OpenCvPipeline {

    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MID,
        RIGHT
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));

    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;


    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean elementMID = rightValue > PERCENT_COLOR_THRESHOLD;

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

        /*
        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
         */

       return mat;
    }

    public Location getLocation(){
        return location;
    }
}
