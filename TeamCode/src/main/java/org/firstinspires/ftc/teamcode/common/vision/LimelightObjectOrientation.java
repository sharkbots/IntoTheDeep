/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample orientation pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.common.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;

/* Qualcomm includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller */
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightObjectOrientation  {

    final Telemetry                 mLogger;

    protected Limelight3A           mWebcam;
    int                             mPipeline;
    int                             mLastProcessed;

    boolean                         mShallUpdateColor;

    List<Sample>                    mWaitingList;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LimelightObjectOrientation(String name, HardwareMap map, Telemetry logger) {

        mLogger             = logger;

        mWebcam             = map.get(Limelight3A.class, name);
        mPipeline           = 6;
        mLastProcessed      = 0;

        mShallUpdateColor   = false;

    }

    /**
     * Start camera streaming
     */
    public void                         start(List<Sample> samples) {

//        mLogger.addLine("starting object orientation pipeline");

        // Format samples into pipeline inputs
        double[] data = new double[ Math.min(1,samples.size()) * 7 + 1];

//        mLogger.addLine(""+samples.size());
        data[0] = mLastProcessed;
        int i_data = 1;
        for (int i_sample = 0; i_sample < Math.min(1,samples.size()); i_sample++) {

            Sample sample = samples.get(i_sample);

            Sample.Color color = sample.color();
            int col = -1;
            if (color == Sample.Color.RED)         { col = 0; }
            else if (color == Sample.Color.BLUE)   { col = 1; }
            else if (color == Sample.Color.YELLOW) { col = 2; }

//            mLogger.addLine("" + sample.index());

            data[i_data] = sample.index(); i_data++;
            data[i_data] = sample.x(); i_data++;
            data[i_data] = sample.y(); i_data++;
            data[i_data] = sample.xMax() - sample.xMin(); i_data++;
            data[i_data] = sample.yMax() - sample.yMin(); i_data++;
            data[i_data] = col; i_data++;
            data[i_data] = sample.area(); i_data ++;
        }

//        mLogger.addLine(""+data.length);

        mLogger.addLine("Sent to pipeline x" + data[2]);
        mLogger.addLine("Sent to pipeline y" + data[3]);


        mWebcam.pipelineSwitch(mPipeline);
        mWebcam.start();
        mWebcam.updatePythonInputs(data);
        mWaitingList = samples;
    }

    /**
     * Retrieve new results
     *
     * @return List of updated samples
     */
    public List<Sample>                     process() {

        List<Sample> result = new ArrayList<>();

        LLResult results = mWebcam.getLatestResult();
        if(results != null) {
//            mLogger.addLine(""+mWaitingList.size());
//            mLogger.addLine(""+results.getPipelineIndex());
            double[] orientations = results.getPythonOutput();

//            mLogger.addLine("" + orientations[0]);
//            mLogger.addLine("" + mLastProcessed);
//            mLogger.addLine("" + orientations[4]);
//            mLogger.addLine("" + orientations[5]);
//            mLogger.addLine("" + orientations[6]);
            if (orientations[0] == mLastProcessed) {
                for (int i_sample = 0; i_sample < (int) ((orientations.length - 1) / 10); i_sample++) {

                    int index = (int) (orientations[i_sample * 10 + 1]);
                    if(index != 0) {
//                        mLogger.addLine("" + index);

                        double orientation = orientations[i_sample * 10 + 5];
                        int col = (int) orientations[i_sample * 10 + 4];

                        Sample.Color color = Sample.Color.UNKNOWN;
                        if (col == 0) {
                            color = Sample.Color.RED;
                        } else if (col == 1) {
                            color = Sample.Color.BLUE;
                        } else if (col == 2) {
                            color = Sample.Color.YELLOW;
                        }

                        for (int j_sample = 0; j_sample < mWaitingList.size(); j_sample++) {
//                            mLogger.addLine("" + mWaitingList.get(j_sample).index());
                            if (index == mWaitingList.get(j_sample).index()) {
                                if (mShallUpdateColor) {
                                    mWaitingList.get(j_sample).color(color);
                                }
                                mWaitingList.get(j_sample).orientation(orientation);
                            }
                        }
                    }

                }

                mLastProcessed ++;
                result = mWaitingList;

            }
        }
        return result;
    }

    public void stop() {
        if(mWebcam != null && mWebcam.isRunning()) { mWebcam.stop(); }
    }

}
