/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample detection pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.common.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightObjectDetection {

    final Telemetry                 mLogger;

    protected Limelight3A           mWebcam;
    int                             mPipeline;
    int                             mSampleId;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LimelightObjectDetection(String name, HardwareMap map, Telemetry logger) {

        mLogger             = logger;

        mWebcam             = map.get(Limelight3A.class, name);
        mPipeline           = 7;

        mSampleId           = 1;

    }

    public boolean                          isReady() {
        LLResult results = mWebcam.getLatestResult();
        return (results != null);
    }

    /**
     * Start camera streaming
     */
    public void                             start() {

        if(mWebcam != null) {
            mLogger.addLine("starting object detection pipeline");
            mWebcam.pipelineSwitch(mPipeline);
            mWebcam.start();
        }
    }

    /**
     * Retrieve new results
     *
     * @return List of detected samples
     */
    public List<Sample>                     process() {

        List<Sample> result = new ArrayList<>();

        if (mWebcam != null) {
            LLResult results = mWebcam.getLatestResult();

            if(results != null) {

                List<LLResultTypes.DetectorResult> detections = results.getDetectorResults();
                for (int i_sample = 0; i_sample < detections.size(); i_sample++) {
                    Sample sample = new Sample(mSampleId, detections.get(i_sample), mLogger);
                    result.add(sample);
                    mSampleId ++;
                }
            }

        }

        return result;
    }
}
