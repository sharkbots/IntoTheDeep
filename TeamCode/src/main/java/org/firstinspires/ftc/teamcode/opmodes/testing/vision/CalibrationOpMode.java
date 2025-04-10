/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep TeleOp mode
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.opmodes.testing.vision;

/* System includes */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.vision.Vision;


@Config
@TeleOp(name = "CalibrationTest", group = "V1")
public class CalibrationOpMode extends LinearOpMode {
    static public double X = 0;
    static public double Y = 0;
    Vision vision;
    
    @Override
    public void runOpMode() {

        try {
            vision = new Vision();
        }
        catch(Exception e){
            telemetry.addLine(e.getMessage());
        }

        waitForStart();

        while(opModeIsActive()) {

            float [] result = vision.limelight.computeGroundPosition(X, Y);

            FtcDashboard.getInstance().getTelemetry().addData("x",result[0]);
            FtcDashboard.getInstance().getTelemetry().addData("y",result[1]);

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}