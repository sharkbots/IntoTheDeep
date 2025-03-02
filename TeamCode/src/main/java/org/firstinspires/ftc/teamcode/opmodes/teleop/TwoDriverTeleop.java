package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.HoldPointCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.CVIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ManualSampleIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ReGrabSampleCommand;
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

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

@Config
@TeleOp(name = "Two Driver Teleop", group = "Teleop")
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

        IS_AUTONOMOUS = false;

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);


        robot.follower.setStartingPose(END_OF_AUTO_POSE);
        robot.follower.setPose(END_OF_AUTO_POSE);

        robot.setProcessorEnabled(robot.sampleDetectionPipeline, true);
        robot.swapYellow();

        SetOperatorGamepadColor();

        // setup hang
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

        // hang
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

        // Switch grabbing mode
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new InstantCommand(()->{
                            GRABBING_MODE = GRABBING_MODE.next();
                            SetOperatorGamepadColor();
                        })
                );

//        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(
//                        new HoldPointCommand(robot.follower, ()->new Pose(robot.follower.getPose().getX()+robot.sampleDetectionPipeline.getCameraXOffset(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading()))
//                );

        // Shoot out intake or transfer
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ConditionalCommand(
                                new TransferCommand(robot),
                                new ConditionalCommand(
                                        new HoverCommand(robot,100),
                                        new InstantCommand(),
                                        () -> !HOLDING_SPECIMEN && !HOLDING_SAMPLE && !INTAKING_SPECIMENS &&
                                                robot.intake.pivotState == IntakeSubsystem.PivotState.TRANSFER),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
                        )

                );


        // Auto grab
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(new CVIntakeCommand(robot)
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(200))),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE && robot.sampleDetectionPipeline.getCameraOffsetMagnitude() != 0
                        ));

        // manual grab
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new ManualSampleIntakeCommand(robot)
                                .alongWith(new InstantCommand(() -> gamepad1.rumble(200))),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL
                        ));


        // ReGrab sample in case of failed grab
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(new ReGrabSampleCommand(robot),
                                new InstantCommand(),
                                () -> robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
                        ));

        // GENERAL RESET
        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new ResetLiftCommand(robot)
                                .alongWith(new ResetIntakeCommand(robot))
                        );


        // Deposit high basket setup
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ConditionalCommand(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET),
                        new InstantCommand(), () -> HOLDING_SAMPLE)
                );

        // Deposit low basket setup
        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new ConditionalCommand(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_LOW_BUCKET),
                        new InstantCommand(), () -> HOLDING_SAMPLE)
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

        // Specimen pick off wall setup
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> INTAKING_SPECIMENS = true)),
                        new InstantCommand(), () -> !INTAKING_SAMPLES && !HOLDING_SAMPLE && !HOLDING_SPECIMEN)
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
                                        new InstantCommand(() -> INTAKING_SPECIMENS = true)),
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
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN)
                                                .alongWith(new InstantCommand(()-> gamepad1.rumble(200))),
                                        new DepositSpecimenCommand(robot),
                                        new WaitCommand(300)
                                ),
                                new InstantCommand(),
                                ()-> robot.lift.liftState == LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP
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

    private void SetOperatorGamepadColor() {
        if (GRABBING_MODE == GRABBING_MODES.SAMPLE){
            gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
        }
        else if (GRABBING_MODE == GRABBING_MODES.SPECIMEN){
            if (ALLIANCE_COLOR == AllianceColor.BLUE) {
                gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
            } else {
                gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
            }
        }
        else {
            gamepad2.setLedColor(0, 255, 0, LED_DURATION_CONTINUOUS);
        }
    }

    @Override
    public void run(){
        //  Runs FTCLib Command Scheduler
        super.run();
        robot.clearBulkCache();
        robot.read();

        if (timer == null){
            robot.reset();
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

        if (Math.abs(gamepad2.left_stick_x)>= 0.2 && robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL){
            robot.intake.setClawRotationDegrees(robot.intake.getClawRotationDegrees() + gamepad2.left_stick_x*4.5);
        }

        // manual extendo control
        //robot.extendoActuator.disableManualPower();
        if (Math.abs(gamepad2.right_stick_y)>= 0.2 &&
                (robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE
                || robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE
                || robot.intake.pivotState == IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL)){
            //robot.extendoActuator.enableManualPower();
            robot.intake.setExtendoTargetTicks((int)(robot.intake.getExtendoPosTicks()+(-gamepad2.right_stick_y*150)));
        }

        // emergency lift override
        if (!robot.lift.isResetting){
            robot.liftActuator.disableManualPower();
        }
        if (gamepad2.left_trigger > 0.75 && gamepad2.right_trigger > 0.75){
            robot.liftActuator.enableManualPower();
            robot.liftActuator.setOverridePower(-gamepad2.left_stick_y);
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
//        // align to closest cardinal point
//        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
//            double alignmentHeading = ((int) Math.round(currentHeading / (Math.PI/2))) * (Math.PI/2);
//
//            if (Math.abs(alignmentHeading-currentHeading) > Math.toRadians(0.5)){
//                rotation = dtHeadingLockOn.calculate(currentHeading, alignmentHeading);
//                rotation = MathUtils.clamp(rotation,-1, 1);
//            }
//        }
        if (robot.sampleDetectionPipeline.getCameraOffsetMagnitude() != 0){

        }


//        if(Math.abs(forward) >= 0.1 && Math.abs(strafe) >= 0.1 && Math.abs(rotation) >= 0.1){
//            //if(!IS_DT_MANUAL_CONTROL)robot.follower.breakFollowing();
//            IS_DT_MANUAL_CONTROL = true;
//            robot.follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
//            robot.previousPose = robot.follower.getPose();
//        }
//        else if (IS_DT_AUTO_ALIGNING){
//            robot.previousPose = robot.follower.getPose();
//        }
//        else{
//            robot.follower.holdPoint(robot.previousPose); //unpushable robot
//            IS_DT_MANUAL_CONTROL = false;
//        }

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
        robot.telemetryA.addData("lift pos", robot.liftActuator.getPosition());
        robot.telemetryA.addData("extendo pos ticks", robot.intake.getExtendoPosTicks());
//        robot.telemetryA.addData("extendo pos inches", robot.intake.getExtendoPosInches());
//        robot.telemetryA.addData("heading", Math.toDegrees(currentHeading));
        robot.telemetryA.addData("runtime", timer.seconds());
        robot.telemetryA.addData("camera y offset", robot.sampleDetectionPipeline.getCameraYOffset());
        //robot.telemetryA.addData("camera x offset (loop)", robot.sampleDetectionPipeline.getCameraXOffset());
//        robot.telemetryA.addData("Pose (during loop, from follower)",String.format(" (%.2f,%.2f,%.2f)", robot.follower.getPose().getX(), robot.follower.getPose().getY(), Math.toDegrees(robot.follower.getPose().getHeading())));
//        robot.telemetryA.addData("Pose (during loop, from pose updater)",String.format(" (%.2f,%.2f,%.2f)", robot.poseUpdater.getPose().getX(), robot.poseUpdater.getPose().getY(), Math.toDegrees(robot.poseUpdater.getPose().getHeading())));
        robot.telemetryA.addData("is busy", robot.follower.isBusy());
        robot.telemetryA.addData("intake pivot state", robot.intake.pivotState);
        //robot.telemetryA.addData("robot voltage", robot.follower.getVoltage());
        robot.telemetryA.addData("lift bottom motor current", robot.liftBottomMotor.getCurrent(CurrentUnit.AMPS));
        robot.telemetryA.addData("lift center motor current", robot.liftCenterMotor.getCurrent(CurrentUnit.AMPS));
        robot.telemetryA.addData("lift top motor current", robot.liftTopMotor.getCurrent(CurrentUnit.AMPS));

        robot.telemetryA.addData("lift bottom motor is over current", robot.liftBottomMotor.isOverCurrent());
        robot.telemetryA.addData("lift center motor is over current", robot.liftCenterMotor.isOverCurrent());
        robot.telemetryA.addData("lift top motor is over current", robot.liftTopMotor.isOverCurrent());


        robot.telemetryA.update();

        loopTime = loop;
    }

    @Override
    public void reset(){
        super.reset();
        robot.telemetryA.addLine("eye of Sauron shutting down...");
        robot.setAutoCameraControls();
        robot.closeCamera();

    }
}
