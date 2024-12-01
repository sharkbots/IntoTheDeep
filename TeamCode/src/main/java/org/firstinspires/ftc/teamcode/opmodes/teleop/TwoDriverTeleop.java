package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;

@Config
@TeleOp(name = "Two Driver Teleop", group = "Teleop")
public class TwoDriverTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.periodic();
        robot.write();

        robot.drivetrain.set(new Pose(-gamepad1.right_stick_x, gamepad1.right_stick_y,
                MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("lift position", robot.liftActuator.getPosition() / 26);
        loopTime = loop;
        telemetry.update();
    }
}
