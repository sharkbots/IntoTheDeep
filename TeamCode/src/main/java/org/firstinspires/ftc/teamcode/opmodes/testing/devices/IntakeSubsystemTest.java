package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

@Disabled
@TeleOp(name = "IntakeSubsystemTest")
public class IntakeSubsystemTest extends CommandOpMode {

    private double loopTime = 0.0;

    private final Robot robot = Robot.getInstance();
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

    private GamepadEx gamepadEx2;

    //TriggerReader leftTrigger, rightTrigger;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTONOMOUS = false;
        Globals.ALLIANCE = Globals.ALLIANCE.BLUE;

        gamepadEx2 = new GamepadEx(gamepad2);

//        leftTrigger = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.LEFT_TRIGGER);
//        rightTrigger = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.RIGHT_TRIGGER);
//
//        leftTrigger.readValue();
//        rightTrigger.readValue();

        robot.init(hardwareMap);
        robot.addSubsystem(robot.intake);
        robot.addSubsystem(robot.lift);

        // shoot out intake
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(new HoverCommand(robot, 100), new InstantCommand(),
                        () ->robot.intake.pivotState == IntakeSubsystem.PivotState.TRANSFER));

        // Grab sample
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ConditionalCommand(new IntakeSampleCommand(robot), new InstantCommand(),
                        () ->robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new InstantCommand(() -> robot.intake.setClawRotationDegrees(robot.intake.getClawRotationDegrees()-10))
                );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(
                        new InstantCommand(() -> robot.intake.setClawRotationDegrees(robot.intake.getClawRotationDegrees()+10))
                );

//        // rotate claw left
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new ConditionalCommand(
//                        new InstantCommand(() -> robot.intake.moveLeft()),
//                        new InstantCommand(),
//                        () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
//                ));
//        // rotate claw right
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new ConditionalCommand(
//                        new InstantCommand(() -> robot.intake.moveRight()),
//                        new InstantCommand(),
//                        () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
//                ));

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
        if (Math.abs(gamepad2.right_stick_y)>= 0.2
                && robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE){
            robot.extendoActuator.enableManualPower();
            robot.extendoActuator.setManualPower(-gamepad2.right_stick_y);
        }

//        leftTrigger.readValue();
//        rightTrigger.readValue();
//        if (leftTrigger.wasJustPressed()) {
//            telemetry.addLine("Left Trigger Pressed!");
//            robot.intake.moveLeft();
//        }
//
//        if (rightTrigger.wasJustPressed()) {
//            telemetry.addLine("Right Trigger Pressed!");
//            robot.intake.moveRight();
//        }

        robot.periodic();
        robot.write();


        double loop = System.nanoTime();
        telemetry.addData("position", robot.extendoActuator.getPosition());
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
