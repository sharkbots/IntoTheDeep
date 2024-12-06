package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;

@Config
@TeleOp(name = "Two Driver Teleop", group = "Teleop")
public class TwoDriverTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;

    public static double dtMinPower = 0.1;
    public static double dtDeadzone = 0.1;
    public static double dtScale = 3.5;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        gamepadEx = new GamepadEx(gamepad1);
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

        // Map button X to reset the lift subsystem
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    robot.lift.reset();
                }));

        // Map button B to set lift state to DEPOSIT_HIGH_BASKET
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET));

        // Map button Y to set lift state to DEPOSIT_HIGH_RUNG
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG));

        // Map left bumper to toggle the claw state
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    LiftSubsystem.ClawState currentClawState = robot.lift.clawState;
                    LiftSubsystem.ClawState newClawState = currentClawState == LiftSubsystem.ClawState.OPEN
                            ? LiftSubsystem.ClawState.CLOSED
                            : LiftSubsystem.ClawState.OPEN;
                    robot.lift.clawState = newClawState;
                    robot.depositClawServo.setPosition(newClawState.getPosition());
                }));

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
        robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.left_stick_x)>= 0.2 &&
                robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING){
            robot.extendoActuator.enableManualPower();
            robot.extendoActuator.setManualPower(1);
        }
        robot.periodic();
        robot.write();

        robot.drivetrain.set(new Pose(-gamepad1.right_stick_x,
                MathUtils.joystickScalar(gamepad1.right_stick_y, dtMinPower, dtDeadzone, dtScale),
                gamepad1.right_stick_x), 0);


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("lift position", robot.liftActuator.getPosition() / 26);
        loopTime = loop;
        telemetry.update();
    }
}
