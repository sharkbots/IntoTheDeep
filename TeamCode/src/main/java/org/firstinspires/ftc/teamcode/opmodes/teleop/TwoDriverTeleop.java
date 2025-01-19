package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.common.utils.math.geometry.Pose;

@Config
@TeleOp(name = "Two Driver Teleop", group = "Teleop")
public class TwoDriverTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();

    private GamepadEx driver;
    private GamepadEx operator;

    private PIDController dtHeadingLockOn;

    private double headingLockTolerance = Math.toRadians(5);
    private boolean flicking = false;


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

        robot.follower.setStartingPose(Globals.END_OF_AUTO_POSE);
        robot.follower.setPose(Globals.END_OF_AUTO_POSE);

        dtHeadingLockOn = new PIDController(0.5, 0, 0);

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

        // Failed specimen pickup
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true)),
                                new InstantCommand(),
                                ()-> robot.lift.getLiftState() == LiftSubsystem.LiftState.HOLDING_SPECIMEN
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
                                new SequentialCommandGroup(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_DOWN)
                                                .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                                        new DepositSpecimenCommand(robot),
                                        new WaitCommand(300)
                                ),
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
            robot.follower.startTeleopDrive();

            timer = new ElapsedTime();
        }
        else if (timer.seconds() > 140 && !notifiedEndgame){
            notifiedEndgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        // manual extendo control
        robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.right_stick_y)>= 0.2 &&
                (robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                || robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)){
            robot.extendoActuator.enableManualPower();
            robot.extendoActuator.setManualPower(-gamepad2.right_stick_y);
        }

        // emergency lift override
        robot.liftActuator.disableManualPower();
        if (gamepad2.left_trigger > 0.75 && gamepad2.right_trigger > 0.75){
            robot.liftActuator.enableManualPower();
            robot.liftActuator.setManualPower(-gamepad2.left_stick_y);
        }
        telemetry.addData("left trigger", gamepad2.left_trigger);
        telemetry.addData("right trigger", gamepad2.right_trigger);



        robot.periodic();
        robot.write();

        double currentHeading = robot.follower.poseUpdater.getPose().getHeading();
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        // slow mode
        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)){
            forward *= 0.3;
            strafe *= 0.3;
            rotation *= 0.3;
        }

//        // flick 180
//        if (driver.wasJustPressed(GamepadKeys.Button.B) || flicking){
//            flicking = true;
//            double alignmentHeading = ((int) Math.round(currentHeading / Math.PI)) * Math.PI;
//
//            if (Math.abs(alignmentHeading-currentHeading) > Math.toRadians(0.5)){
//                rotation = dtHeadingLockOn.calculate(currentHeading, alignmentHeading);
//                rotation = MathUtils.clamp(rotation,-1, 1);
//            }
//            else {
//                flicking = false;
//            }
//            robot.follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
//            robot.follower.update();
//        }
        // align to closest cardinal point
        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            double alignmentHeading = ((int) Math.round(currentHeading / (Math.PI/2))) * (Math.PI/2);

            if (Math.abs(alignmentHeading-currentHeading) > Math.toRadians(0.5)){
                rotation = dtHeadingLockOn.calculate(currentHeading, alignmentHeading);
                rotation = MathUtils.clamp(rotation,-1, 1);
            }
            robot.follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
            robot.follower.update();
        }
        else {
            robot.drivetrain.set(new Pose(-strafe,
                    forward,
                    -rotation), 0);
        }


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("lift position", robot.liftActuator.getPosition());
        telemetry.addData("heading", Math.toDegrees(currentHeading));
        loopTime = loop;
        telemetry.update();
    }
}
