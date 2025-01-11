package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ReGrabSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ResetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.ResetLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.math.geometry.Pose;

@Config
@TeleOp(name = "Two Driver Teleop", group = "Teleop")
public class TwoDriverTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();

    private GamepadEx driver;
    private GamepadEx operator;

    //TriggerReader leftTrigger, rightTrigger;

    private double loopTime = 0.0;
    private ElapsedTime timer;

    public static double dtMinPower = 0.1;
    public static double dtDeadzone = 0.1;
    public static double dtScale = 3.5;

    private boolean notifiedEndgame = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        // rotate claw right
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new ConditionalCommand(
                                new InstantCommand(() -> robot.intake.moveLeft()),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                        ));
        // rotate claw right
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(() -> robot.intake.moveRight()),
                        new InstantCommand(),
                        () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                ));

        // Shoot out intake
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ConditionalCommand(
                                new TransferCommand(robot),
                                new ConditionalCommand(new HoverCommand(robot,100), new InstantCommand(),
                                        () -> !Globals.HOLDING_SPECIMEN && !Globals.HOLDING_SAMPLE && !Globals.INTAKING_SPECIMENS &&
                                                robot.intake.pivotState == IntakeSubsystem.PivotState.TRANSFER),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
                        )

                );

        // Grab sample
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(new IntakeSampleCommand(robot)
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(200))),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                            ));

        // ReGrab sample in case of failed grab
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(new ReGrabSampleCommand(robot),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
                        ));

        // Reset lift
        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new ResetLiftCommand(robot)
                                .alongWith(new ResetIntakeCommand(robot))
                        );


        // Deposit high basket setup
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ConditionalCommand(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET),
                        new InstantCommand(), () -> Globals.HOLDING_SAMPLE)
                );

        // Deposit sample
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new DepositSampleCommand(robot)
                                        .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET
                                        || robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_LOW_BASKET
                        )
                );

        // Specimen pick off wall setup
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true)),
                        new InstantCommand(), () -> !Globals.INTAKING_SAMPLES && !Globals.HOLDING_SAMPLE && !Globals.HOLDING_SPECIMEN)
                );

        // Intake specimen
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                            new IntakeSpecimenCommand(robot)
                                    .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                            new InstantCommand(),
                            ()-> robot.lift.getLiftState() == LiftSubsystem.LiftState.INTAKE_SPECIMEN
                        )
                );

        // Deposit high rung setup
        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP),
                        new InstantCommand(),
                        () -> robot.lift.getLiftState() == LiftSubsystem.LiftState.HOLDING_SPECIMEN)
                );

        // Deposit specimen
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new DepositSpecimenCommand(robot)
                                        .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP
                        )
                );


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

        if (timer == null){
            robot.reset();

            timer = new ElapsedTime();
        }
        else if (timer.seconds() > 140 && !notifiedEndgame){
            notifiedEndgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.right_stick_y)>= 0.2 &&
                (robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                || robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)){
            robot.extendoActuator.enableManualPower();
            robot.extendoActuator.setManualPower(-gamepad2.right_stick_y);
        }

        robot.periodic();
        robot.write();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;
        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            forward *= 0.3;
            strafe *= 0.3;
            rotation *= 0.3;
        }
        robot.drivetrain.set(new Pose(strafe,
                forward,
                rotation), 0);

//        leftTrigger.readValue();
//        if (leftTrigger.wasJustPressed() && robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING) {
//            telemetry.addLine("Left Trigger Pressed!");
//            robot.intake.moveLeft();
//        }
//
//        rightTrigger.readValue();
//        if (rightTrigger.wasJustPressed() && robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING) {
//            telemetry.addLine("Right Trigger Pressed!");
//            robot.intake.moveRight();
//        }


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("lift position", robot.liftActuator.getPosition());
        loopTime = loop;
        telemetry.update();
    }
}
