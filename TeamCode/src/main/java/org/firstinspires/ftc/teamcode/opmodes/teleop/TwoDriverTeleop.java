package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ResetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.HangCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.ResetLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.common.vision.Sample;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
@TeleOp(name = "Two Driver Teleop", group = "1 Teleop")
public class TwoDriverTeleop extends CommandOpMode {

    private DashboardPoseTracker dashboardPoseTracker;

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
    private boolean hangDone = false;
    private boolean readyToLetGo = false;

    private int matchLength = 120;
    private int increaseCounter = 0;


    @Override
    public void initialize() {
        super.reset();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        IS_AUTONOMOUS = false;

        GRABBING_MODES.set(GRABBING_MODES.SPECIMEN);
        UpdateOperatorGamepadColor();

        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);


        robot.follower.setStartingPose(END_OF_AUTO_POSE);
        robot.follower.setPose(END_OF_AUTO_POSE);

//        robot.setProcessorEnabled(robot.sampleDetectionPipeline, true);
//        robot.swapYellow();


        // GENERAL RESET
        operator.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(
                        new ResetLiftCommand(robot)
                                .alongWith(new ResetIntakeCommand(robot))
                );

        // Switch between sample and specimen mode
        operator.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
                .whenPressed(
                        new InstantCommand(() -> {
                            GRABBING_MODES.next();
                            UpdateOperatorGamepadColor();
                        })
                );

        // Setup hang
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> readyToLetGo = false),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.LVL2_ASCENT_SETUP),
                                        new InstantCommand(()-> readyToLetGo = true)
                                ),
                                new InstantCommand(),
                                () -> notifiedEndgame
                        )
                );

        // Hang
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new HangCommand(robot)
                                ),
                                new InstantCommand(),
                                () -> robot.lift.liftState == LiftSubsystem.LiftState.LVL2_ASCENT_SETUP
                        )
                );


        // Shoot out intake (bucket mode)
        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        new ConditionalCommand(
                                new HoverCommand(robot,200),
                                new InstantCommand(),
                                () -> !HOLDING_SAMPLE /*&& !HOLDING_SPECIMEN && !INTAKING_SPECIMENS*/ && GRABBING_MODES.current() == GRABBING_MODES.SAMPLE &&
                                        !INTAKING_SAMPLES)
                );

        // Shoot out intake (spec mode)
        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        new ConditionalCommand(
                                new HoverCommand(robot,500),
                                new InstantCommand(),
                                () -> !INTAKING_SAMPLES &&
                                        robot.intake.pivotState == IntakeSubsystem.PivotState.TRANSFER && GRABBING_MODES.current() == GRABBING_MODES.SPECIMEN)
                );

        // Also reset lift if needed
        operator.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        new ConditionalCommand(
                                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
                                new InstantCommand(),
                                ()->!HOLDING_SAMPLE && !HOLDING_SPECIMEN && !INTAKING_SPECIMENS
                        )
                );


//
//        // Transfer (sample mode)
//        operator.getGamepadButton(GamepadKeys.Button.CROSS)
//                .whenPressed(
//                        new ConditionalCommand(
//                                new TransferCommand(robot),
//                                new InstantCommand(),
//                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
//                                        && GRABBING_MODES.current() == GRABBING_MODES.SAMPLE
//                        )
//                );
//
//        // Transfer (specimen mode)
//        operator.getGamepadButton(GamepadKeys.Button.CROSS)
//                .whenPressed(
//                        new ConditionalCommand(
//                                new TransferCommand(robot).andThen(
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
//                                ),
//                                new InstantCommand(),
//                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
//                                        && GRABBING_MODES.current() == GRABBING_MODES.SPECIMEN
//                        )
//                );

        // Drop off sample in OZ and setup specimen intake
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
//                                        new InstantCommand(()->robot.lift.updateState(LiftSubsystem.LiftState.INTAKE_SPECIMEN))
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> {
                                            INTAKING_SPECIMENS = true;
                                            HOLDING_SAMPLE = false;
                                        })),
                                new InstantCommand(),
                                () -> robot.lift.liftState == LiftSubsystem.LiftState.INTAKE_SPECIMEN && HOLDING_SAMPLE
                        )
                );


        // Sample grab (sample mode)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new IntakeSampleCommand(robot).interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get())
                                        .alongWith(new InstantCommand(() -> gamepad1.rumble(200)))
                                        .andThen(new TransferCommand(robot))
//                                            new ParallelRaceGroup(
//                                                    new TransferCommand(robot).andThen(
//                                                            new InstantCommand(()-> INTAKE_JUST_CANCELLED = false)
//                                                    ),
//                                            new WaitUntilCommand(()->operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get()).andThen(
//                                                    new InstantCommand(()-> INTAKE_JUST_CANCELLED = true))
//                                            )
//                                        )
                                    .interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get())
                                ,
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL && !HOLDING_SAMPLE && !HOLDING_SPECIMEN && !INTAKING_SPECIMENS
                                && GRABBING_MODES.current() == GRABBING_MODES.SAMPLE
                        )
                );

        // sample grab (spec mode)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new IntakeSampleCommand(robot).interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get())
                                        .alongWith(new InstantCommand(() -> gamepad1.rumble(200)))
                                        .andThen(
                                                new ParallelRaceGroup(
                                                        new TransferCommand(robot).interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get())
                                                                .andThen(
                                                                        new SequentialCommandGroup(
                                                                                new WaitCommand(130),
                                                                                new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
                                                                                        .alongWith(
                                                                                                new WaitCommand(400),
                                                                                                new InstantCommand(() -> robot.depositClawServo.setPosition(0.67)),
                                                                                                new InstantCommand(() -> INTAKE_JUST_CANCELLED = false)
                                                                                        )
                                                                        )
                                                                ),
                                                        new WaitUntilCommand(() -> operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get())
                                                                .andThen(new InstantCommand(() -> INTAKE_JUST_CANCELLED = true))
                                                )
                                        )
                                ,
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL
                                        && !HOLDING_SAMPLE && !HOLDING_SPECIMEN && !INTAKING_SPECIMENS
                                        && GRABBING_MODES.current() == GRABBING_MODES.SPECIMEN
                        )
                );


        // Deposit high basket setup
        operator.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(new ConditionalCommand(
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(new InstantCommand(
                                ()-> robot.lift.setClawState(LiftSubsystem.ClawState.MICRO_OPEN)
                        )),
                        new InstantCommand(), () -> HOLDING_SAMPLE && GRABBING_MODES.current() == GRABBING_MODES.SAMPLE)
                );

        // Deposit low basket setup
        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new ConditionalCommand(
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_LOW_BUCKET).andThen(
                                new InstantCommand(()->robot.lift.setClawState(LiftSubsystem.ClawState.CLOSED))
                        ),
                        new InstantCommand(), () -> HOLDING_SAMPLE && GRABBING_MODES.current() == GRABBING_MODES.SAMPLE)
                );

        // Deposit sample
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new DepositSampleCommand(robot)
                                        .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET
                                        || robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_LOW_BUCKET
                        )
                );

        // OZ Specimen pickup setup
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> INTAKING_SPECIMENS = true)),
                        new InstantCommand(), () -> !HOLDING_SAMPLE && !HOLDING_SPECIMEN)
                );



        // Intake specimen
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                            new IntakeSpecimenCommand(robot)
                                    .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                            new InstantCommand(),
                            ()-> robot.lift.getLiftState() == LiftSubsystem.LiftState.INTAKE_SPECIMEN && !HOLDING_SAMPLE
                        )
                );

        // Failed specimen pickup
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> INTAKING_SPECIMENS = true)),
                                new InstantCommand(),
                                ()-> robot.lift.getLiftState() == LiftSubsystem.LiftState.HOLDING_SPECIMEN
                        )
                );

        // Deposit high rung setup
        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(new ConditionalCommand(
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.depositClawServo.setPosition(0.63)),
                                        new WaitCommand(400),
                                        new InstantCommand(()-> robot.depositClawServo.setPosition(0.66))
                                )
                        ),
                        new InstantCommand(),
                        () -> robot.lift.getLiftState() == LiftSubsystem.LiftState.HOLDING_SPECIMEN ||
                        robot.lift.getLiftState() == LiftSubsystem.LiftState.PUSHING_SPECIMEN)
                );

        AtomicBoolean readyToReleaseSpec = new AtomicBoolean(false);
        // Deposit specimen: 1. Hang it
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN).withTimeout(250).andThen(new InstantCommand(()-> readyToReleaseSpec.set(true)))
                                        .alongWith(
                                                new InstantCommand(()-> {
                                                    gamepad1.rumble(200);
                                                    robot.depositClawServo.setPosition(DEPOSIT_CLAW_MICRO_OPEN_POS);
                                                }),
                                                new HoverCommand(robot, 50).interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).get())
                                        ),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP
                        )
                );

        // Deposit specimen: 2. Open the claw
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(readyToReleaseSpec::get),
                                        new DepositSpecimenCommand(robot),
                                        new WaitCommand(200),
                                        new ResetLiftCommand(robot).interruptOn(() -> operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).get())
                                ),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN
                        )
                );


        // Deposit specimen: 3. Driver sweeper mode
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new LiftCommand(robot, LiftSubsystem.LiftState.PUSHING_SPECIMEN)
                                        .alongWith(new InstantCommand(()-> {
                                            gamepad1.rumble(200);
                                            robot.depositClawServo.setPosition(DEPOSIT_CLAW_CLOSED_POS);
                                        })),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP ||
                                        robot.lift.liftState == LiftSubsystem.LiftState.HOLDING_SPECIMEN
                        )
                );



        robot.read();
        super.run();
        while (opModeInInit()) {
            robot.telemetryA.addData("right trigger", gamepad2.right_trigger);
            robot.telemetryA.addLine("Robot Initialized.");
            robot.telemetryA.update();
        }
    }

    private void UpdateOperatorGamepadColor() {
        int[] rgb = GRABBING_MODES.getControllerColor();
        gamepad2.setLedColor(rgb[0], rgb[1], rgb[2], LED_DURATION_CONTINUOUS);
    }

    @Override
    public void run(){
        //  Runs FTCLib Command Scheduler
        super.run();
        robot.clearBulkCache();
        robot.read();

        if (timer == null){
            //robot.reset();
            robot.follower.startTeleopDrive();

            timer = new ElapsedTime();
        }
        else if (timer.seconds() > (matchLength-5) && !notifiedEndgame){
            notifiedEndgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
        if (gamepad2.left_stick_button && gamepad2.right_stick_button){
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            notifiedEndgame = true;
        }

        int maxExtendoPower = 125;
        double clawRotationFactor = 3.5;
        if (gamepad2.left_trigger > 0.95){
            maxExtendoPower = maxExtendoPower * 2;
           // clawRotationFactor = clawRotationFactor / 1.25;
        }

        if (Math.abs(gamepad2.left_stick_x)>= 0.2 && robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL){
            robot.intake.setClawRotationDegrees(robot.intake.getClawRotationDegrees() + gamepad2.left_stick_x*clawRotationFactor);
        }

        // manual extendo control
        //robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.right_stick_y)>= 0.025 && INTAKING_SAMPLES){
            robot.intake.setExtendoTargetTicks((int)(robot.intake.getExtendoPosTicks()+
                    MathUtils.extendoJoystickScalar(-gamepad2.right_stick_y, 0.025, 0.4, 0.3, 0.99999, 0.6, 1.5, maxExtendoPower)));
        }

        // manual lift control when depositing samples
        if (Math.abs(gamepad2.left_stick_y)>= 0.2 && robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET){
            double factor = gamepad2.left_stick_y < 0 ? 45 : 25;
            robot.lift.setLiftTargetPosTicks(((int)(robot.liftActuator.getPosition()-gamepad2.left_stick_y*factor)));
        }

        // emergency lift override
        if (!robot.lift.isResetting){
            robot.liftActuator.disableManualPower();
        }
        if (gamepad2.left_trigger > 0.75 && gamepad2.right_trigger > 0.75){
            robot.liftActuator.enableManualPower();
            robot.liftActuator.setOverridePower(-gamepad2.left_stick_y);
            robot.liftTopEncoder.reset();
        }

        //robot.telemetryA.addData("left trigger", gamepad2.left_trigger);
        //robot.telemetryA.addData("right trigger", gamepad2.right_trigger);

        // Slowly come down from hang
        if (readyToLetGo && timer.seconds() > (matchLength+3) && !hangDone){
            //robot.liftActuator.enableManualPower();
            increaseCounter ++;
            robot.liftActuator.setTargetPosition(robot.liftActuator.getTargetPosition()+1);
            //robot.liftActuator.setOverridePower(-0.5);
            if (robot.liftActuator.getPosition() > POST_BUZZER_HANG_RELEASE_HEIGHT){
                hangDone = true;
            }
        }

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

        // TODO: fix using turn to

        robot.follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
//        if (!IS_DT_AUTO_ALIGNING){
//            robot.follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
//        }
        //gamepad button mapped to -> follower.followPath(), auto=true;
        //robot.follo(during loop, from followewer.setTeleOpMovementVectors(forward, strafe, rotation, true);

        robot.periodic();
        robot.write();
        robot.follower.update();


        double loop = System.nanoTime();
        robot.telemetryA.addData("hz ", 1000000000 / (loop - loopTime));
        robot.telemetryA.addData("Operator right joystick y value", -operator.getRightY());
        robot.telemetryA.addData("lift pos", robot.liftActuator.getPosition());
        robot.telemetryA.addData("extendo pos ticks", robot.intake.getExtendoPosTicks());
//        robot.telemetryA.addData("extendo pos inches", robot.intake.getExtendoPosInches());
//        robot.telemetryA.addData("heading", Math.toDegrees(currentHeading));
        robot.telemetryA.addData("runtime", timer.seconds());

        robot.vision.detect(Sample.Color.YELLOW);
        if(!robot.vision.samples().isEmpty()) {
            robot.telemetryA.addLine("samples found");
            float[] offsets = robot.vision.selected();
            Sample result = robot.vision.selectedSample();
            if (result != null) {
                robot.telemetryA.addData("Closest result (Pixels): ", String.format("%.1f, %.1f",result.x(), result.y()));
                robot.telemetryA.addData("Closest result (Offset): ", String.format("%.1f, %.1f, %.1f", offsets[0], offsets[1], offsets[2]));
                robot.telemetryA.addData("Closest result color: ", result.color());
            }
        }
        // Vision testing
        //FtcDashboard.getInstance().sendImage(robot.vision.draw(320,240));

        //robot.telemetryA.addData("current pose", robot.follower.getPose());
       // robot.telemetryA.addData("is busy", robot.follower.isBusy());
        //robot.telemetryA.addData("intake pivot state", robot.intake.pivotState);
        robot.telemetryA.addData("grabbing mode", GRABBING_MODES.current());
        robot.telemetryA.addData("intaking samples", INTAKING_SAMPLES);
        robot.telemetryA.addData("intaking specimens", INTAKING_SPECIMENS);
        robot.telemetryA.addData("holding sample", HOLDING_SAMPLE);
        robot.telemetryA.addData("holding specimen", HOLDING_SPECIMEN);
        robot.telemetryA.addData("dumping spec mode: ", SPEC_DUMPING_MODE);
        //robot.telemetryA.addData("robot voltage", robot.follower.getVoltage());


        robot.telemetryA.update();

        loopTime = loop;
    }

    @Override
    public void reset(){
        super.reset();
        robot.telemetryA.addLine("eye of Sauron shutting down...");
    }

    @Override
    public void end(){
        //robot.vision.stop();
    }
}
