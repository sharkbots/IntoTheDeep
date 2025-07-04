package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.google.ar.core.exceptions.SessionUnsupportedException;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.CVIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.ManualSampleIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.SetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.vision.Sample;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.PreloadSampleCycleGenerator;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SubSampleCycleGenerator;



import java.util.ArrayList;

@Config
@Autonomous(name = "🟡 6-Sample Auto", group = "1 blue auto", preselectTeleOp = "Two Driver Teleop")
public class SixSampleAuto extends CommandOpMode {
    private Telemetry telemetryA;

    private final Robot robot = Robot.getInstance();

    ConfigMenu menu;
    boolean alreadyCompiled = false;
    GamepadEx operator;
    private final Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;

    PreloadSampleCycleGenerator preloadSampleCyclePathGen;
    SubSampleCycleGenerator subSampleCyclePathGen;

    @Override
    public void initialize() {
        super.reset();
        Globals.IS_AUTONOMOUS = true;
        Globals.GRABBING_MODES.set(Globals.GRABBING_MODES.SAMPLE);

        timer.reset();

        operator = new GamepadEx(gamepad2);

        robot.setTelemetry(telemetry);
        robot.telemetryA.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);

        menu = new ConfigMenu(operator, robot);
        menu.setConfigurationObject(new Globals.SampleAutonomousConfig());
        operator.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(menu::backupCurrentField);

        robot.init(hardwareMap);

        robot.follower.setMaxPower(1);

        preloadSampleCyclePathGen = new PreloadSampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        subSampleCyclePathGen = new SubSampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        robot.reset();
        robot.lift.setClawState(LiftSubsystem.ClawState.MICRO_OPEN);

        while(opModeInInit()){
            menu.periodic();
//            MathUtils.clamp(Globals.SampleAutonomousConfig.samp1X, Globals.sampleAutoStartPose.getX(), 12.5);
//            MathUtils.clamp(Globals.SampleAutonomousConfig.samp1Y, 12, 42.0);

            //MathUtils.clamp(Globals.SampleAutonomousConfig.samp2X, -11, 2);
            //MathUtils.clamp(Globals.SampleAutonomousConfig.samp2Y, -8, 8);


            if (menu.isLocked() && !alreadyCompiled){
                alreadyCompiled = true;

                Globals.ALLIANCE_COLOR = Globals.SampleAutonomousConfig.allianceColor;

                subSampleCyclePathGen.addSubSampleLocation(new Pose(Globals.SampleAutonomousConfig.samp1X+72, Globals.SampleAutonomousConfig.samp1Y+72, Point.CARTESIAN), 1);
                subSampleCyclePathGen.addSubSampleLocation(new Pose(Globals.SampleAutonomousConfig.samp2X+72, Globals.SampleAutonomousConfig.samp2Y+72, Point.CARTESIAN), 2);
                subSampleCyclePathGen.addSubSampleLocation(new Pose(Globals.SampleAutonomousConfig.samp2X+72, Globals.SampleAutonomousConfig.samp2Y+72, Point.CARTESIAN), 3);


                generatePaths();
                generateSchedule();
                robot.telemetryA.addLine("recompiled");
            }
            else if (!menu.isLocked()){
                alreadyCompiled = false;
                super.reset();
            }
            robot.telemetryA.addData("right trigger", operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        }
    }


    private void generatePaths(){
        paths.clear();
        //robot.follower.setStartingPose(allianceColor.convert(Globals.sampleAutoStartPose, Pose.class));
        robot.follower.setPose(allianceColor.convert(Globals.sampleAutoStartPose, Pose.class));

        // PATH 0 - sample preload
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(Globals.sampleAutoStartPose),
                                        new Point(15.351, 126.486, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Globals.sampleAutoStartPose.getHeading(), Math.toRadians(315))
                        .addPath(
                                new BezierLine(
                                        new Point(15.351, 126.486, Point.CARTESIAN),
                                        allianceColor.convert(Globals.bucketPose, Point.class)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(315))
                        //.setPathEndTValueConstraint()
                        .build()
        );

        // PATH 1&2 - inside sample
        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.INSIDE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.INSIDE));

        // PATH 3&4 - middle sample
        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.MIDDLE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.MIDDLE));

        // PATH 5&6 - outside sample
        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.OUTSIDE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.OUTSIDE));

        // PATH 7&8 - sample from sub
        paths.add(subSampleCyclePathGen.getSubPickupPath(1));
        paths.add(subSampleCyclePathGen.getSubDepositPath(1));

        // PATH 9&10 - sample from sub
        paths.add(subSampleCyclePathGen.getSubPickupPath(2));
        paths.add(subSampleCyclePathGen.getSubDepositPath(2));

        // PATH 11&12 - sample from sub
        paths.add(subSampleCyclePathGen.getSubPickupPath(3));
        paths.add(subSampleCyclePathGen.getSubDepositPath(3));

        // PATH 13 - park
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        Globals.bucketPose,
                                        new Pose(62.486, 116.324, Point.CARTESIAN),
                                        new Pose(59.243, 92.541, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
                        .addParametricCallback(0.7, ()-> robot.follower.setMaxPower(0.3))
                        .setPathEndTValueConstraint(0.99)
                        .setPathEndTimeoutConstraint(250)
                        .build()
        );



    }

    private void generateSchedule() {
        schedule(
                new RunCommand(robot::clearChubCache),
                new RunCommand(robot::read),
                new RunCommand(robot::write),
                new RunCommand(robot::periodic),
                new RunCommand(robot.follower::update),

                new SequentialCommandGroup(
                        // Deposit preload sample
                        new FollowPathChainCommand(robot.follower, paths.get(0)).setHoldEnd(false)
                                .alongWith(
                                    new SequentialCommandGroup(
                                            new WaitCommand(100), // see if we can put this to 0
                                            new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                    )
                                ),

                        new WaitCommand(600-400),
                        new DepositSampleCommand(robot),
                        new WaitCommand(200),

//                        // Pickup OZ sample
//                        new ConditionalCommand(
//                            new SequentialCommandGroup(
//                            new InstantCommand(()-> robot.follower.setMaxPower(0.9)),
//                            new ParallelCommandGroup(
//                                    new FollowPathChainCommand(robot.follower, paths.get(8)).setHoldEnd(true),
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(300),
//                                            new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
//                                                    new HoverCommand(robot, 100)
//                                            )
//                                    )
//                            ),
//                            new WaitCommand(Globals.SampleAutonomousConfig.waitOZinSeconds*1000),
//                            new IntakeSampleCommand(robot),
//                            new TransferCommand(robot),
//
//                            // Deposit OZ sample
//                            new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(600),
//                                            new FollowPathChainCommand(robot.follower, paths.get(9)).disableUseIsBusy()
//                                                    .setCompletionThreshold(0.9)
//                                    )
//
//                            ),
//                            new WaitCommand(150),
//                            new DepositSampleCommand(robot)
//                            ),
//                            new InstantCommand(),
//                            ()->Globals.SampleAutonomousConfig.grabSecondPreload
//                        ),

                        // Pickup inside sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                            new FollowPathChainCommand(robot.follower, paths.get(1)).setHoldEnd(true),
                            new SequentialCommandGroup(
                                    new WaitCommand(300),
                                    new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                            new HoverCommand(robot, 1090.7+100)
                                    )
                            )
                        ),
                        new IntakeSampleCommand(robot),
                        new TransferCommand(robot),


                        // Deposit inside sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
                                new SequentialCommandGroup(
                                        new FollowPathChainCommand(robot.follower, paths.get(2)).disableUseIsBusy()
                                                .setCompletionThreshold(0.95)
                                )

                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot),


                        // Pickup middle sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new FollowPathChainCommand(robot.follower, paths.get(3)).setHoldEnd(true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300+200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                                new HoverCommand(robot, 1140.7+100)
                                        )
                                )
                        ),
                        new IntakeSampleCommand(robot),
                        new TransferCommand(robot),


                        // Deposit middle sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200+400),
                                        new FollowPathChainCommand(robot.follower, paths.get(4)).disableUseIsBusy()
                                                .setCompletionThreshold(0.95)
                                )

                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot),


                        // Pickup outside sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new FollowPathChainCommand(robot.follower, paths.get(5)).setHoldEnd(true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                                new HoverCommand(robot, 1415.7-100, 44.77)
                                        )
                                )
                        ),

                        new IntakeSampleCommand(robot),
                        new ParallelCommandGroup(
                                new TransferCommand(robot).andThen(new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300+1000),
                                        new FollowPathChainCommand(robot.follower, paths.get(6)).disableUseIsBusy()
                                                .setCompletionThreshold(0.9)
                                )
                        ),
                        new WaitCommand(250),
                        new DepositSampleCommand(robot),

                        // Pickup 5th sample from sub
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathChainCommand(robot.follower, paths.get(7)).alongWith(
                                new SequentialCommandGroup(
                                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0),
                                        new WaitCommand(500-200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        //new WaitCommand(500),
                        new DeferredCommand(()->new CVIntakeCommand(robot, Sample.Color.YELLOW), null),

                        // Deposit 5th sample from sub
                        new FollowPathChainCommand(robot.follower, paths.get(8)).alongWith(
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        new WaitCommand(50),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )
                        ),
                        new WaitCommand(600-300),
                        new DepositSampleCommand(robot),
                        //new WaitCommand(150),

                  //      new InstantCommand(() -> {robot.vision.stop();robot.vision.start();})


                        // Pickup 6th sample from sub
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathChainCommand(robot.follower, paths.get(9)).alongWith(
                                new SequentialCommandGroup(
                                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0),
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        //new WaitCommand(500),
                        new DeferredCommand(()->new CVIntakeCommand(robot, Sample.Color.YELLOW), null),

                        // Deposit 6th sample from sub
                        new FollowPathChainCommand(robot.follower, paths.get(10)).alongWith(
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        new WaitCommand(50),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )

                        ),
                        new WaitCommand(600-300),
                        new DepositSampleCommand(robot),
                        //new WaitCommand(150),

                        new FollowPathChainCommand(robot.follower, paths.get(13)).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.LVL1_ASCENT)
                                )
                        )

                        /*
                        // Pickup 7th sample from sub
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathChainCommand(robot.follower, paths.get(11)).alongWith(
                                new SequentialCommandGroup(
                                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0),
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        new CVIntakeCommand(robot, Sample.Color.YELLOW),

                        // Deposit 6th sample from sub
                        new FollowPathChainCommand(robot.follower, paths.get(12)).alongWith(
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )

                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot)

                         */

                )
            );
    }

    @Override
    public void run(){
        super.run();

        //FtcDashboard.getInstance().sendImage(robot.vision.draw(320,240));
        if(!robot.vision.samples().isEmpty())
        {
            robot.telemetryA.addLine("Samples found");
            float[] offsets = robot.vision.selected();
            Sample result = robot.vision.selectedSample();
            if (result != null) {
                robot.telemetryA.addData("Closest result (Pixels): ", String.format("%.1f, %.1f",result.x(), result.y()));
                robot.telemetryA.addData("Closest result (Offset): ", String.format("%.1f, %.1f, %.1f", offsets[0], offsets[1], offsets[2]));
                robot.telemetryA.addData("Closest result color: ", result.color());
            }
        }

        robot.telemetryA.addData("Robot Pose", robot.follower.getPose());
        double loop = System.nanoTime();
        robot.telemetryA.addData("hz ", 1000000000 / (loop - loopTime));
        robot.telemetryA.addLine(robot.follower.getPose().toString());
        robot.telemetryA.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        robot.telemetryA.addData("Lift pos", robot.liftActuator.getPosition());
        robot.telemetryA.addData("Lift motor powers", robot.liftActuator.getPower());
        robot.telemetryA.addData("t value", robot.follower.getCurrentTValue());
        robot.telemetryA.update();

        loopTime = loop;
        Globals.END_OF_AUTO_POSE = robot.follower.poseUpdater.getPose();
    }

    @Override
    public void reset(){
        super.reset();
    }
}