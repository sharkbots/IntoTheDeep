package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

@Disabled
@TeleOp(name = "LiftSubsystemTest", group="2 tests")
public class LiftSubsystemTest extends CommandOpMode {

    private double loopTime = 0.0;

    private final Robot robot = Robot.getInstance();
    private LiftSubsystem lift;

    private GamepadEx gamepadEx2;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTONOMOUS = false;
        //Globals.ALLIANCE_FIXED_VAL = Globals.ALLIANCE_FIXED_VAL.BLUE;

        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        lift = new LiftSubsystem();
        robot.addSubsystem(robot.lift);

        // Map button X to reset the lift subsystem
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    lift.reset();
                }));

        // Map button B to set lift state to DEPOSIT_HIGH_BASKET
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET));

        // Map button Y to set lift state to DEPOSIT_HIGH_RUNG
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP));

        // Map left bumper to toggle the claw state
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    LiftSubsystem.ClawState currentClawState = lift.clawState;
                    LiftSubsystem.ClawState newClawState = currentClawState == LiftSubsystem.ClawState.OPEN
                            ? LiftSubsystem.ClawState.CLOSED
                            : LiftSubsystem.ClawState.OPEN;
                    lift.clawState = newClawState;
                    robot.depositClawServo.setPosition(newClawState.getPosition());
                }));

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
        robot.periodic();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("position", robot.liftActuator.getPosition());
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
