package org.firstinspires.ftc.teamcode.opmodes.testing.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Sample Detection Op Mode", group = "Testing")
public class SampleDetectionOpMode extends LinearOpMode {
    Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setProcessorEnabled(robot.sampleDetectionPipeline, true);
        robot.swapYellow();
        telemetry.addData("camera state", robot.getCameraState());
        robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE);

        robot.telemetryA.addData("Exposure mode", robot.exposureControl.getMode());
        robot.telemetryA.addData("White balance mode", robot.whiteBalanceControl.getMode());
        robot.telemetryA.addData("Exposure length", robot.exposureControl.getExposure(TimeUnit.MILLISECONDS));
        robot.telemetryA.addData("White balance color", robot.whiteBalanceControl.getWhiteBalanceTemperature());
        robot.telemetryA.addData("Gain", robot.gainControl.getGain());
        robot.telemetryA.update();

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            robot.clearBulkCache();
            if(gamepad1.cross){
//            robot.telemetryA.addData("Exposure length", robot.exposureControl.getExposure(TimeUnit.MILLISECONDS));
//            robot.telemetryA.addData("Exposure length min", robot.exposureControl.getMinExposure(TimeUnit.MILLISECONDS));
//            robot.telemetryA.addData("Exposure length max", robot.exposureControl.getMaxExposure(TimeUnit.MILLISECONDS));
//            robot.telemetryA.addData("White balance color", robot.whiteBalanceControl.getWhiteBalanceTemperature());
//            robot.telemetryA.addData("White balance min", robot.whiteBalanceControl.getMinWhiteBalanceTemperature());
//            robot.telemetryA.addData("White balance max", robot.whiteBalanceControl.getMaxWhiteBalanceTemperature());
//            robot.telemetryA.addData("Gain", robot.gainControl.getGain());
                robot.setManualCameraControls();
            }
            if(gamepad1.circle){
                robot.setAutoCameraControls();
            }
        robot.telemetryA.addData("Exposure mode", robot.exposureControl.getMode());
        robot.telemetryA.addData("White balance mode", robot.whiteBalanceControl.getMode());
        robot.telemetryA.addData("Exposure length", robot.exposureControl.getExposure(TimeUnit.MILLISECONDS));
        robot.telemetryA.addData("White balance color", robot.whiteBalanceControl.getWhiteBalanceTemperature());
        robot.telemetryA.addData("Gain", robot.gainControl.getGain());
        robot.telemetryA.update();
        }

        robot.telemetryA.addLine("opmode is stopped woohoo");
        robot.telemetryA.update();

        robot.setAutoCameraControls();
        robot.closeCamera();
        robot.telemetryA.addLine("Camera is closed");
        robot.telemetryA.update();
        robot.kill();
    }
}
