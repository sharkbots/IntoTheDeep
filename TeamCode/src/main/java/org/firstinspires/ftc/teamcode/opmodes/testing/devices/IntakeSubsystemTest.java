package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

@TeleOp(name = "IntakeSubsystemTest")
public class IntakeSubsystemTest extends CommandOpMode {

    private double loopTime = 0.0;

    private final Robot robot = Robot.getInstance();
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    private GamepadEx gamepadEx2;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.ALLIANCE = Globals.ALLIANCE.BLUE;

        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.addSubsystem(robot.intake);
        robot.addSubsystem(robot.lift);

        // shoot out intake
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(new HoverCommand(robot), new InstantCommand(),
                        () ->robot.intake.pivotState == IntakeSubsystem.PivotState.TRANSFER));

        // Grab sample
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ConditionalCommand(new IntakeCommand(robot), new InstantCommand(),
                        () ->robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING));

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run();
        robot.clearBulkCache();
        robot.read();
        robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.left_stick_x)>= 0.2){
            robot.extendoActuator.enableManualPower();
            robot.extendoActuator.setManualPower(gamepad2.left_stick_y);
        }

        robot.periodic();
        robot.write();


        double loop = System.nanoTime();
        telemetry.addData("position", robot.extendoActuator.getPosition());
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
