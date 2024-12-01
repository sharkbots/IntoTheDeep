package org.firstinspires.ftc.teamcode.opmodes.testing.vision;

import android.graphics.Canvas;

import org.opencv.core.Point;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Processor implements VisionProcessor {

    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Imgproc.GaussianBlur(frame, frame, new Size(9.0, 9.0), 75, 75);
        //Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(frame, new Scalar(90, 90, 20), new Scalar(135, 255, 255), frame);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int numContours = contours.size();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[numContours];
        RotatedRect[] rectangle = new RotatedRect[numContours];
        double maxArea = 0;
        int currentBiggestRectangle = -1;
        for (int i = 0; i < numContours; i++){
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);

            double rectangleRatio = Math.max(rectangle[i].size.width,rectangle[i].size.height)/Math.min(rectangle[i].size.width,rectangle[i].size.height);

            if (rectangleRatio > 2){
                Point[] vertices = new Point[4];
                rectangle[i].points(vertices);
                List<MatOfPoint> box = new ArrayList<>();
                box.add(new MatOfPoint(vertices));
                Imgproc.drawContours(frame, box, -1, new Scalar(255, 0, 0), 3, 0);
                continue;
            }

        }



        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
