/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Sample detection processing
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.common.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;

/* Android includes */
import android.graphics.Bitmap;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Opencv includes */
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;


public class SamplesDetection  {

    public enum Mode {
        NONE,
        DETECT,
        ORIENT}

    static  float                  sCameraYOffset = 4.53f;

    final Telemetry                 mLogger;

    LimelightObjectDetection        mDetection;
    LimelightObjectOrientation      mOrientation;
    Calibration                     mCalibration;

    Mode                            mMode;
    Sample.Color                    mColor;

    List<Sample>                    mOngoing;
    List<Sample>                    mConsolidated;
    Sample                          mSelected;

    int                             mImageIndex;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  SamplesDetection(String name, HardwareMap map, Telemetry logger) {

        mLogger             = logger;
        mMode               = Mode.NONE;

        mDetection          = new LimelightObjectDetection(name, map, logger);
        mOrientation        = new LimelightObjectOrientation(name, map, logger);
        mCalibration        = new Calibration();
        mImageIndex         = 0;
        mConsolidated       = new ArrayList<>();
        mOngoing            = new ArrayList<>();
        mSelected           = null;
        mColor              = Sample.Color.UNKNOWN;

    }
    
    private void                             selectColor(String color) {

        mColor = Sample.Color.UNKNOWN;
        if(color != null) {
            if (color.equals("red")) { mColor = Sample.Color.RED; }
            else if (color.equals("blue")) { mColor = Sample.Color.BLUE; }
            else if (color.equals("yellow")) { mColor = Sample.Color.YELLOW; }
        }
    }

    public List<Sample> samples() { return mConsolidated; }


    public float[] selected() {
        float[] result = new float[3];
        if(mSelected != null) {
            result[0] = (float)(mSelected.distanceX() * 1.1);
            result[1] = (float)mSelected.distanceY();
            result[2] = (float)mSelected.orientation() - 90;
        }
        return result;
    }

    public Sample selectedSample() { return mSelected; }

    /**
     * Start sample detection
     */
    public void start() {
        mDetection.start();
        mImageIndex = 0;
        mCalibration.initialize();
        mMode = Mode.DETECT;
    }

    /**
     * Detect new samples
     */
    public void detect(String color) {

        selectColor(color);
        mLogger.addLine("**DETECT COLOR** "+mColor);

        if (mMode == Mode.DETECT) {

            List<Sample> detections = mDetection.process();
            //mLogger.addLine("" + detections.size());
            if (!detections.isEmpty()) {
                mOngoing.clear();
                for (Sample sample : detections) {
                    float[] ground = mCalibration.computeGroundPosition(320 - sample.x(), sample.y());
                    sample.distanceX(ground[1]);
                    sample.distanceY(-ground[0] + sCameraYOffset);
                    mOngoing.add(sample);
                }
                mOngoing.sort(Comparator.comparingDouble(s -> mergedRanking(s, mColor)));

                mOngoing.subList(1, mOngoing.size()).clear();

                mLogger.addData("**LIST SIZE** ", mOngoing.size());

//                mLogger.addLine("Switching to orientation");
                mOrientation.start(mOngoing);
                mMode = Mode.ORIENT;
            }
        }
        else if (mMode == Mode.ORIENT) {
            List<Sample> oriented = mOrientation.process();
            if (!oriented.isEmpty()) {
                mLogger.addData("**SELECT ORIENTED SIZE** ", mColor);

//                mLogger.addLine("Switching back to detection");
                mConsolidated.clear();
                mConsolidated.addAll(oriented);
                mSelected = mConsolidated.get(0);
                mDetection.start();
                mMode = Mode.DETECT;
                mOngoing.clear();
            }
        }

//        mLogger.addLine("Processing image " + mImageIndex);
        mImageIndex++;
    }

    private static double distanceRanking(Sample s, double x0, double y0) {
        double dx = s.x() - x0;
        double dy = s.y() - y0;
        return dx * dx + dy * dy;
    }

    private static double confidenceRanking(Sample s) {
        return s.confidence();
    }

    private static double mergedRankingOnClaw(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = s.distanceY();
        }
        return result;
    }

    private static double mergedRanking(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = s.distanceY();
        }
        return result;
    }

    /**
     * Create an empty bitmap, black content, to add overlays to
     *
     * @param width : width of the image to create
     * @param height : height of the image to create
     *
     * @return An android bitmap of a black imafge with samples overlays
     * **/
    public Bitmap                           draw(int width, int height) {
        // Create a blank image (black)
        Mat frame = new Mat(height, width, CvType.CV_8UC3, new Scalar(0, 0, 0));

        for(Sample sample : mConsolidated) {

            Point topLeft = new Point(sample.xMin(), sample.yMin());
            Point bottomRight = new Point(sample.xMax(), sample.yMax());

            Scalar color = new Scalar(255,255,255);
            if(sample.color() == Sample.Color.RED) { color = new Scalar(255,0,0); }
            if(sample.color() == Sample.Color.BLUE) { color = new Scalar(0,0,255); }
            if(sample.color() == Sample.Color.YELLOW) { color = new Scalar(255,255,0); }

            Point cross1 = new Point(Math.max(0,sample.x() - 10), sample.y());
            Point cross2 = new Point(Math.min(width - 1,sample.x() + 10), sample.y());
            Point cross3 = new Point(sample.x(), Math.max(0,sample.y() - 10));
            Point cross4 = new Point(sample.x(),Math.min(height - 1,sample.y() + 10));

            String text = sample.index() + " - " +  String.format("%.2f",sample.confidence()) + " - " + (int)(sample.orientation());
            Size size = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,1,null);
            Point text1 = new Point(sample.xMin(), Math.max(0,sample.yMin() - size.height));
            Point text2 = new Point(Math.min(width - 1,sample.xMin() + size.width), sample.yMin());

            Imgproc.rectangle(frame, topLeft, bottomRight, color, 2);
            Imgproc.line(frame, cross1, cross2, color, 2);
            Imgproc.line(frame, cross3, cross4, color, 2);
            Imgproc.rectangle(frame, text1, text2, color,Core.FILLED);
            if(sample.index() == mSelected.index()) {
                Imgproc.putText(frame, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            }
            else {
                Imgproc.putText(frame, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 1);
            }
        }

        // Convert to Bitmap to show in Android
        Bitmap result = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, result);
        return result;
    }

    /** Log consolidated samples to the dashboard **/
    public void                             log()
    {

        StringBuilder result = new StringBuilder();

        result.append("<p>" + mMode + "</p>");

        result.append("<details open style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> CONSOLIDATED SAMPLES </summary>\n");
        result.append("<ul>\n");
        for (Sample sample : mConsolidated) {
            result.append(sample.logHTML());
        }
        result.append("</ul>\n");
        result.append("</details>\n");

        result.append("<details open style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> ON GOING SAMPLES </summary>\n");
        result.append("<ul>\n");
        for (Sample sample : mOngoing) {
            result.append(sample.logHTML());
        }
        result.append("</ul>\n");
        result.append("</details>\n");

        //mLogger.addLine(result.toString());
    }

    public void stop() {
        if(mDetection != null) { mDetection.stop(); }
        if(mOrientation != null) { mOrientation.stop(); }
    }

}
