/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Estimate ground position from pixel position based
   on a set of reference points and homography estimation
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.common.vision;

/* System includes */
import java.util.List;
import java.util.Arrays;

/* Open CV includes */
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Core;
import org.opencv.calib3d.Calib3d;

public class Calibration {
    List<Point> sGroundCoordinates = Arrays.asList(
            new Point(0,4), new Point(0,7), new Point(0,10), new Point(0,13), new Point(0,16), new Point(-3,4), new Point(-3,7), new Point(-3,10), new Point(-3,13), new Point(-3,16), new Point(-6,7), new Point(-6,10), new Point(-6,13), new Point(-6,16), new Point(-9,16), new Point(3,4), new Point(3,7), new Point(3,10), new Point(3,13), new Point(3,16), new Point(3,19), new Point(6,7), new Point(6,10), new Point(6,13), new Point(6,16), new Point(6,19), new Point(9,16));
    List<Point> sPixelPoints  = Arrays.asList(
            new Point(160,57), new Point(160,118), new Point(160,161), new Point(160,192), new Point(160,218), new Point(72,55), new Point(87,117), new Point(98,160), new Point(106,193), new Point(112,218), new Point(14,117), new Point(35,159), new Point(52,193), new Point(64,219), new Point(17,220), new Point(250,56), new Point(233,119), new Point(222,162), new Point(214,195), new Point(208,219), new Point(205,238), new Point(308,120), new Point(286,163), new Point(270,195), new Point(257,218), new Point(248,237), new Point(306,219)   );

    Mat         mHomography;

    /** Constructor */
    public Calibration()
    {
        mHomography = null;
    }

    /** Compute homography from image **/
    public void                             initialize()
    {
        MatOfPoint2f imgMat = new MatOfPoint2f();
        imgMat.fromList(sPixelPoints);
        MatOfPoint2f groundMat = new MatOfPoint2f();
        groundMat.fromList(sGroundCoordinates);
        mHomography =  Calib3d.findHomography(imgMat, groundMat);
    }

    /**
     * Compute ground position from an input pixel
     * @param x : pixel x position in upper left corner reference
     * @param y : pixel y position in upper left corner reference
     * @return Ground position associated to the pixel
     */
    public float[] computeGroundPosition( double x, double y) {

        Point pixel = new Point(x,y);
        Mat src = new Mat(1, 1, CvType.CV_32FC2);
        src.put(0, 0, new float[]{(float) pixel.x, (float) pixel.y});
        Mat dst = new Mat();
        Core.perspectiveTransform(src, dst, mHomography);
        float[] result = new float[2];
        dst.get(0, 0, result);
        return result;
    }


}
