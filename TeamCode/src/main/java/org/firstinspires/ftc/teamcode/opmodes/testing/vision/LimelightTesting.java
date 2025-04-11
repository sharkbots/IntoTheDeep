package org.firstinspires.ftc.teamcode.opmodes.testing.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;


@TeleOp(name = "LimelightTesting", group = "V1")
public class LimelightTesting extends LinearOpMode {
    Robot robot = Robot.getInstance();
    LLResultTypes.DetectorResult result;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            robot.telemetryA.addLine("in loop");
            LLStatus status = robot.vision.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            robot.vision.limelight.setLatestResults("yellow");

            if (robot.vision.limelight.getLatestResults() != null) {
                robot.telemetryA.addLine("detection is not null");
                result = robot.vision.limelight.getClosestResult();
                float[] offsets = robot.vision.limelight.getClosestOffset();
                if (result != null) {
                    robot.telemetryA.addData("Closest result (Pixels): ", result.getTargetXPixels() + ", " + result.getTargetYPixels());
                    robot.telemetryA.addData("Closest result (Offset): ", offsets[0] + ", " + offsets[1] + ", " + offsets[2]);
                    robot.telemetryA.addData("Closest result color: ", result.getClassName());
                }
            }


            robot.telemetryA.update();
        }

    }
}