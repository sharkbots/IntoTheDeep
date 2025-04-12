/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localizers final test tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.opmodes.testing.vision;

/* System includes */
import java.util.List;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Algorithms includes */
import org.firstinspires.ftc.teamcode.common.vision.SamplesDetection;
import org.firstinspires.ftc.teamcode.common.vision.Sample;

@Config
@TeleOp(name = "SamplesDetectionTest", group = "Test")
public class SamplesDetectionTest extends LinearOpMode {

    static public  String                       sColor = "yellow";

    private SamplesDetection                    mDetection;

    @Override
    public void runOpMode() {

        try {

            mDetection = new SamplesDetection("Limelight3A", hardwareMap,FtcDashboard.getInstance().getTelemetry());
            mDetection.start();
            FtcDashboard.getInstance().getTelemetry().update();

            waitForStart();

            FtcDashboard.getInstance().getTelemetry().clear();

            while(opModeIsActive()) {

                mDetection.detect(sColor);
                mDetection.log();
                List<Sample> samples = mDetection.samples();
                FtcDashboard.getInstance().sendImage(mDetection.draw(320, 240));
                FtcDashboard.getInstance().getTelemetry().update();

            }

        }
        catch(Exception e) {
            FtcDashboard.getInstance().getTelemetry().addLine(e.toString());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }



}