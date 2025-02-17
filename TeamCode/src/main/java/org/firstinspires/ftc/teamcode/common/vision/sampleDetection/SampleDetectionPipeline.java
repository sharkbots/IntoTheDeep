package org.firstinspires.ftc.teamcode.common.vision.sampleDetection;

import static java.lang.Math.PI;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
public class SampleDetectionPipeline implements VisionProcessor {
    Robot robot = Robot.getInstance();
    public double ARM_ANGLE_NOT_NEEDED = 0;
    public double ARM_HEIGHT_NOT_NEEDED = 0;
    public static int retVal = 0;


    List<MatOfPoint> contours = new ArrayList<>();
    public static double RUH = 5, RLH = 160, RS = 90, RV = 70, BH = 101, BUH = 120, BS = 50, BV = 75, YH = 11, YUH = 33, YS = 80, YV = 150, AREA_RATIO_WEIGHT = -0.4, UPPIES = .5, MIN_AREA = 7000,FOR_MULT=.7,
            FOR_CONST = 3;
    public static int UPPER_THRESH = 130, LOWER_THRESH = 120, YUPPER_THRESH = 165, YLOWER_THRESH = 60, KERNEL_SIZE = 8, YELLOW_KERNEL_SIZE = 8;
    Mat hsv = new Mat();
    Mat mask = new Mat(), mask2 = new Mat(), closedEdges = new Mat(), edges = new Mat();
    Mat kernel = new Mat();
    Mat colorMask = new Mat();
    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat(), maskedImage = new Mat();

    public static double AREA_THRESH = .82, FCL = 1, UP_TOLERANCE = 0.85, DOWN_TOLERANCE = 0.8, CLASSUP_TOL = 0.5, CLASSDOWN_TOL = 0.3;
    double objectWidth = 3.5;  // Inches
    double objectHeight = 1.5;  // Inches

    // Define the 3D coordinates of the object corners in the object coordinate space
    MatOfPoint3f objectPoints = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));
    MatOfPoint3f objectPoints1 = new MatOfPoint3f(
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, -1.5),
            new Point3(objectWidth / 2, -objectHeight / 2, -1.5),
            new Point3(objectWidth / 2, objectHeight / 2, 0));
    MatOfPoint3f objectPoints2 = new MatOfPoint3f(
            new Point3(objectWidth / 2, -objectHeight / 2, -1),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(objectWidth / 2, objectHeight / 2, -1)
    );

    MatOfPoint3f rlObjectPoints;

    Point[] orderedRectPoints;
    Mat rvec = new Mat();
    Mat tvec = new Mat();
    MatOfPoint2f contour2f = new MatOfPoint2f();
    private volatile Double[] closestCenter = {0.0, 0.0, 0.0, 0.0};

    public static int color = 0;

    /*
     * Camera Calibration Parameters
     */
    //Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
   // MatOfDouble distCoeffs = new MatOfDouble();
    RotatedRect minAreaRect;

    public SampleDetectionPipeline(){
//        double fx = 396.39 * FCL; // Replace with your camera's focal length in pixels
//        double fy = 396.39 * FCL;
//        double cx = 646.424; // Replace with your camera's principal point x-coordinate (usually image width / 2)
//        double cy = 360.884; // Replace with your camera's principal point y-coordinate (usually image height / 2)
//        cameraMatrix.put(0, 0,
//                fx, 0, cx,
//                0, fy, cy,
//                0, 0, 1);
      //  distCoeffs = new MatOfDouble(0.00981454, -0.0292495, 0.00489965, -0.000205308, -4.06933e-05);
    }

    public void resetCenter() {
        closestCenter = new Double[]{0.0, 0.0, 0.0, 0.0};
    }

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        closestCenter = new Double[]{0.0, 0.0, 0.0, 0.0};
        Imgproc.GaussianBlur(input, input, new Size(CAMERA_GAUSSIAN, CAMERA_GAUSSIAN), 0);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar rlFilt = new Scalar(RLH, RS, RV),
                ruFilt = new Scalar(180, 255, 255),
                rllFilt = new Scalar(0, RS, RV),
                rulFilt = new Scalar(RUH, 255, 255),
                blFilt = new Scalar(BH, BS, BV),
                buFilt = new Scalar(BUH, 255, 255),
                ylFilt = new Scalar(YH, YS, YV),
                yuFilt = new Scalar(YUH, 255, 255);

        input.copyTo(boundingImage);  // More memory-efficient
        // Threshold pixels that are not in range
        if (color == 0) {
            Core.inRange(hsv, rlFilt, ruFilt, mask);
            Core.inRange(hsv, rllFilt, rulFilt, mask2);
            Core.bitwise_or(mask, mask2, colorMask);
        } else if (color == 1)
            Core.inRange(hsv, blFilt, buFilt, colorMask);
        else
            Core.inRange(hsv, ylFilt, yuFilt, colorMask);

        maskedImage = new Mat();
        // Fill in remaining pixels from the threshold
        Core.bitwise_and(input, input, maskedImage, colorMask);

        edges = new Mat();
        // Apply Canny edge detection
        if (color != 2) {
            Imgproc.Canny(maskedImage, edges, LOWER_THRESH, UPPER_THRESH);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(KERNEL_SIZE, KERNEL_SIZE));
            closedEdges = new Mat();
            Imgproc.dilate(edges, closedEdges, kernel);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);
        } else {
            Imgproc.Canny(maskedImage, edges, YLOWER_THRESH, YUPPER_THRESH);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            closedEdges = new Mat();
            Imgproc.dilate(edges, closedEdges, kernel);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);
        }

        contours = new ArrayList<>();
        Imgproc.findContours(closedEdges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


        ArrayList<Double[]> centerCoords = contoursToCoords();

        for(Double[] coord: centerCoords) {
            Point center = new Point(coord[0].intValue(), coord[1].intValue());
            Imgproc.circle(boundingImage, center, 5, new Scalar(255, 0, 255), 4);
            //Imgproc.putText(boundingImage, center.toString(), center, Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 255), 2);
        }

        if (!centerCoords.isEmpty()){
            closestCenter = closestCenter(centerCoords);
            Point closestCenterPoint = new Point(closestCenter[0].intValue(), closestCenter[1].intValue());
            if (cameraInRange()){
                Imgproc.circle(boundingImage, closestCenterPoint, 5, new Scalar(0, 255, 0), 4);
                Imgproc.putText(boundingImage, this.closestCenter.toString(), closestCenterPoint, Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 255, 0), 2);
            }
            else {
                Imgproc.circle(boundingImage, closestCenterPoint, 5, new Scalar(0, 0, 255), 4);
                Imgproc.putText(boundingImage, this.closestCenter.toString(), closestCenterPoint, Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 0, 255), 2);

            }
        }



        //input.copyTo(hsv);

        if (retVal == 0)
            boundingImage.copyTo(hsv);
        else if (retVal == 1)
            maskedImage.copyTo(hsv);
        else if (retVal == 2)
            edges.copyTo(hsv);
        else
            closedEdges.copyTo(hsv);
//
//        closedEdges.release();
//        edges.release();
//        colorMask.release();
//       // hsv.release();
//        mask.release();
//        mask2.release();
//        maskedImage.release();
//        hierarchy.release();
//        hierarchy.release();
//        boundingImage.release();

//        Core.rotate(maskedImage, maskedImage, Core.ROTATE_180);
//        Bitmap maskedBmp = Bitmap.createBitmap(maskedImage.cols(), maskedImage.rows(), Bitmap.Config.ARGB_8888);
//        Utils.matToBitmap(maskedImage, maskedBmp);
//        FtcDashboard.getInstance().sendImage(maskedBmp);

        Core.rotate(hsv, hsv, Core.ROTATE_180);
        Bitmap inputBmp = Bitmap.createBitmap(hsv.cols(), hsv.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(hsv, inputBmp);
        FtcDashboard.getInstance().sendImage(inputBmp);

        return input;

    }

    public synchronized void setCenter(Double[] newCenter) {
        closestCenter = newCenter;
    }

    public synchronized Double[] getCenter() {
        return closestCenter;
    }

    double[] convertToDoubleArray(Double[] wrapperArray) {
        double[] primitiveArray = new double[wrapperArray.length];

        for (int i = 0; i < wrapperArray.length; i++) {
            primitiveArray[i] = wrapperArray[i]; // Auto-unboxing
        }

        return primitiveArray;
    }

//    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords) {
//        ArrayList<Double[]> matchedCenters = new ArrayList<>();
//        double minDist = 1000;
//        int coord = 0;
//        for (int i = 0; i < colorCoords.size(); i++) {
//
//            matchedCenters.add(colorCoords.get(i));
//            double[] relCent = convertToDoubleArray(colorCoords.get(i));
//            relCent[0] = (relCent[2] * sin(ARM_ANGLE_NOT_NEEDED * PI / 180) + relCent[0] * cos(ARM_ANGLE_NOT_NEEDED * PI / 180) - FOR_CONST)*FOR_MULT;
//            if (relCent[0]*relCent[0]+relCent[1]*relCent[1] < minDist) {
//                coord = matchedCenters.size() - 1;
//                minDist = relCent[0]*relCent[0]+relCent[1]*relCent[1];
//            }
//        }
//        if (matchedCenters.isEmpty())
//            return new Double[]{100.0, 100.0, 100.0, 100.0};
//        return matchedCenters.get(coord);
//    }

    public ArrayList<Double[]> contoursToCoords() {
        ArrayList<Double[]> centers = new ArrayList<>();
        // Set acceptable aspect ratio range
        double minAspectRatio = 3.5 / 1.5 - DOWN_TOLERANCE;
        double maxAspectRatio = 3.5 / 1.5 + UP_TOLERANCE;
        double minAreaThreshold = 10000;  // Minimum area threshold
        // Iterate over contours
        for (MatOfPoint contour : contours) {
            // Filter out small contours based on area
            if (Imgproc.contourArea(contour) < MIN_AREA) {
                continue;
            }

            // Approximate the contour to a polygon
            contour2f = new MatOfPoint2f(contour.toArray());
            minAreaRect = Imgproc.minAreaRect(contour2f);

            if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 && Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {

                Point[] box = new Point[4];
                minAreaRect.points(box);

                Point[] orded = orderPoints(box);
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;
                    if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {
                        // Draw the bounding rectangle on the image

                        double rotRectAngle = minAreaRect.angle;
                        if (minAreaRect.size.width < minAreaRect.size.height) {
                            rotRectAngle += 90;
                        }

                        // Compute the angle and store it
                        double angle = (rotRectAngle);

                        centers.add(new Double[]{(box[0].x+box[2].x)/2, (box[0].y+box[2].y)/2, 0.0, angle}); // add x offset
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(255, 0, 255), 4);
                        }
                        }
                    }
                }
            }

        return centers;
    }

    public Double[] closestCenter(ArrayList<Double[]> coords){
        double smallestXOffset = Math.abs(Robot.getParallaxXCm(coords.get(0)[0].intValue(), coords.get(0)[1].intValue()) - Robot.getParallaxXCm(CAMERA_STREAM_WIDTH/2, coords.get(0)[1].intValue()));
        int closestCenterID = 0;
        for (int i = 1; i < coords.size(); i++){
            double xOffset = Math.abs(Robot.getParallaxXCm(coords.get(i)[0].intValue(), coords.get(0)[1].intValue()) - Robot.getParallaxXCm(CAMERA_STREAM_WIDTH/2, coords.get(0)[1].intValue()));
            if (xOffset<smallestXOffset) {
                smallestXOffset = xOffset;
                closestCenterID = i;
            }

        }
        return coords.get(closestCenterID);
    }

//    public Double[] getCameraOffset(Double[] closestCenter){
//        return new Double[] {(Robot.getParallaxXCm(closestCenter[0].intValue()) - Robot.getParallaxXCm(CAMERA_STREAM_WIDTH/2))/2.54,
//                (Robot.getParallaxYCm(closestCenter[1].intValue()) - Robot.getParallaxYCm(CAMERA_STREAM_HEIGHT/2))/2.54, closestCenter[3]/2.54, (closestCenter[4]-(90+0))%180}; // +0 is current angle claw
//    }

    //TODO: REFACTOR TO CV MANAGER CLASS

    public boolean cameraInRange(){
        return Math.abs(getCameraYOffset()) < 0.5 && Math.abs(getCameraHeadingOffsetDegrees()) < (20.0); /*getCameraXOffset() < 0.3*/
    }
    public double getCameraXOffset(){
        return (Robot.getParallaxXCm(this.closestCenter[0].intValue(), this.closestCenter[1].intValue()) - Robot.getParallaxXCm(CAMERA_STREAM_WIDTH/2, this.closestCenter[1].intValue()))/2.54;
    }

    public double getCameraYOffset(){
        return (Robot.getParallaxYCm(this.closestCenter[1].intValue()) - Robot.getParallaxYCm(CAMERA_STREAM_HEIGHT/2))/2.54;
    }

    public double getCameraZOffset(){
        return this.closestCenter[2]/2.54;
    }

    public double getCameraHeadingOffsetDegrees(){
        return((this.closestCenter[3]-(90+0))%180); // +0 is current angle claw
    }

    private static double distance(Point p1, Point p2) {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    public static Point[] orderCorner(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }
        Point[] sortedByDistance = {pts[0], null, null};
        double d1 = distance(pts[0], pts[1]);
        double d2 = distance(pts[0], pts[2]);
        double d3 = distance(pts[0], pts[3]);
        Point[] orderedPts = new Point[4];
        if (d1 < d2 && d1 < d3) {
            //  3 0 1 2, 3 1 0 2, 2 1 0 3, 2 0 1 3
            int[] shorts = {0, 1};
            int[] longs = {2, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else if (d2 < d3) {
            int[] shorts = {0, 2};
            int[] longs = {1, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else {
            int[] shorts = {0, 3};
            int[] longs = {1, 2};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }


        return orderedPts;
    }

    // Helper method to calculate the dot product of two vectors
    double dotProduct(Point p1, Point p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }

    // Helper method to calculate the magnitude of a vector
    double magnitude(Point p) {
        return sqrt(p.x * p.x + p.y * p.y);
    }

    // Helper method to calculate the angle between two vectors represented by Points
    double calculateAngle(Point p1, Point p2) {
        double dotProd = dotProduct(p1, p2);
        double magP1 = magnitude(p1);
        double magP2 = magnitude(p2);
        double cosTheta = dotProd / (magP1 * magP2);
        return Math.acos(cosTheta) * (180.0 / PI); // Return the angle in degrees
    }

    public static Point[] orderPoints(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Calculate the center of the frame
        double centerX = 400; // 1280
        double centerY = 300; // 720
        Point center = new Point((int) centerX, (int) centerY);

        // Calculate distances from each point to the center
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = distance(center, pts[i]);
        }

        // Sort points by proximity to center
        Point[] sortedByDistance = Arrays.copyOf(pts, 4);
        Arrays.sort(sortedByDistance, Comparator.comparingDouble(p -> distance(center, p)));

        // Start with the two closest points
        Point[] orderedPts = new Point[4];
        orderedPts[0] = sortedByDistance[0]; // Closest point
        orderedPts[1] = sortedByDistance[1]; // Second closest point

        // Remaining points
        Point thirdPoint = sortedByDistance[2];
        Point fourthPoint = sortedByDistance[3];

        // Determine the order of the remaining points in clockwise manner
        if (isCC(thirdPoint, orderedPts[0], orderedPts[1], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[0], orderedPts[1], fourthPoint};
        } else if (isCC(thirdPoint, orderedPts[1], orderedPts[0], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[1], orderedPts[0], fourthPoint};

        } else if (isCC(fourthPoint, orderedPts[1], orderedPts[0], thirdPoint)) {
            orderedPts = new Point[]{fourthPoint, orderedPts[1], orderedPts[0], thirdPoint};

        } else {
            orderedPts = new Point[]{fourthPoint, orderedPts[0], orderedPts[1], thirdPoint};

        }

        return orderedPts;
    }


    public static boolean isCC(Point p1, Point p2, Point p3, Point p4) {
        return isCCW(p1, p2, p3) && isCCW(p1, p3, p4) &&
                isCCW(p2, p3, p4) && isCCW(p1, p2, p4);
    }

    private static boolean isCCW(Point a, Point b, Point c) {
        // Calculate the cross product of vector AB and AC
        double crossProduct = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return crossProduct > 0; // Returns true if the points are in counter-clockwise order
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }


    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return new Scalar(0, 0, 255);
            case "Yellow":
                return new Scalar(255, 255, 0);
            default:
                return new Scalar(255, 0, 0);
        }
    }

    public void setColor(int color) {
        this.color = color;
    }

    public int getColor() {
        return color;
    }
}