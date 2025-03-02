package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
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
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.PreloadSampleCycleGenerator;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SubSampleCycleGenerator;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;


import java.util.ArrayList;

@Config
@Autonomous(name = "A🔵 FIVE Auto", group = "blue auto", preselectTeleOp = "Two Driver Teleop")
public class FiveSampleAuto extends CommandOpMode {
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

    public void generatePaths(){
        paths.clear();
        //robot.follower.setStartingPose(allianceColor.convert(Globals.sampleAutoStartPose, Pose.class));
        robot.follower.setPose(allianceColor.convert(Globals.sampleAutoStartPose, Pose.class));

        // sample preload
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(6.595, 111.355, Point.CARTESIAN),
                                        new Point(15.351, 126.486, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                        .addPath(
                                new BezierLine(
                                        new Point(15.351, 126.486, Point.CARTESIAN),
                                        allianceColor.convert(Globals.bucketPose, Point.class)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(315))
                        .setPathEndTimeoutConstraint(0.9)
                        .build()
        );

        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.INSIDE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.INSIDE));


        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.MIDDLE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.MIDDLE));

        paths.add(preloadSampleCyclePathGen.getSamplePath(PreloadSampleCycleGenerator.SampleLocation.OUTSIDE));
        paths.add(preloadSampleCyclePathGen.getBucketPath(PreloadSampleCycleGenerator.SampleLocation.OUTSIDE));

        paths.add(subSampleCyclePathGen.getSubPickupPath(1));
        paths.add(subSampleCyclePathGen.getSubDepositPath(1));

        paths.add(subSampleCyclePathGen.getSubPickupPath(2));
        paths.add(subSampleCyclePathGen.getSubDepositPath(2));


//        // park
//        paths.add(
//                robot.follower.pathBuilder()
//                        .addPath(
//                                // Line 6
//                                new BezierCurve(
//                                        allianceColor.convert(new Point(12.386, 128.573, Point.CARTESIAN)),
//                                        allianceColor.convert(new Point(41.514, 117.189, Point.CARTESIAN)),
//                                        allianceColor.convert(new Point(59.027, 122.162, Point.CARTESIAN))
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
//                        .addPath(
//                                // Line 7
//                                new BezierLine(
//                                        allianceColor.convert(new Point(59.027, 122.162, Point.CARTESIAN)),
//                                        allianceColor.convert(new Point(62.054, 91.527, Point.CARTESIAN))
//                                )
//                        )
//                        .setConstantHeadingInterpolation(Math.toRadians(90))
//                        .addParametricCallback(0.9, ()-> robot.follower.setMaxPower(1))
//                        .setPathEndTValueConstraint(0.99)
//                        .setPathEndTimeoutConstraint(250)
//                        .build()
//        );
    }
    @Override
    public void initialize() {
        super.reset();
        Globals.IS_AUTONOMOUS = true;
        Globals.ALLIANCE = Globals.AllianceColor.BLUE;
        operator = new GamepadEx(gamepad2);

        timer.reset();

        robot.setTelemetry(telemetry);
        robot.telemetryA.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);
        menu = new ConfigMenu(operator, robot);
        menu.setConfigurationObject(new Globals.SampleAutonomousConfig());
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(menu::backupCurrentField);

        robot.init(hardwareMap);

        robot.setProcessorEnabled(robot.sampleDetectionPipeline, true);
        robot.swapYellow();

        robot.follower.setMaxPower(1);

        preloadSampleCyclePathGen = new PreloadSampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        subSampleCyclePathGen = new SubSampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

//        generatePaths();
//        generateSchedule();


        robot.reset();
        robot.lift.updateState(LiftSubsystem.ClawState.CLOSED);
        while(opModeInInit()){
            menu.periodic();
            MathUtils.clamp(Globals.SampleAutonomousConfig.samp1X, -11, 2);
            MathUtils.clamp(Globals.SampleAutonomousConfig.samp2X, -11, 2);

            MathUtils.clamp(Globals.SampleAutonomousConfig.samp1Y, -8, 8);
            MathUtils.clamp(Globals.SampleAutonomousConfig.samp2Y, -8, 8);


            if (menu.isLocked() && !alreadyCompiled){
                alreadyCompiled = true;

                subSampleCyclePathGen.addSubSampleLocation(new Pose(Globals.SampleAutonomousConfig.samp1X+72, Globals.SampleAutonomousConfig.samp1Y+72, Point.CARTESIAN), 1);
                subSampleCyclePathGen.addSubSampleLocation(new Pose(Globals.SampleAutonomousConfig.samp2X+72, Globals.SampleAutonomousConfig.samp2Y+72, Point.CARTESIAN), 2);

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
//                                .alongWith(
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
//                                ),
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(100), // see if we can put this to 0
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )
                                ),

                        new WaitCommand(200),
                        new DepositSampleCommand(robot),


                        // Pickup inside sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                            new FollowPathChainCommand(robot.follower, paths.get(1)).setHoldEnd(true),
                            new SequentialCommandGroup(
                                    new WaitCommand(300),
                                    new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                            new HoverCommand(robot, 1000)
                                    )
                            ),
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(()->robot.follower.getVelocityMagnitude() < CV_INTAKE_MAX_VELOCITY),
                                    new WaitUntilCommand(()->robot.sampleDetectionPipeline.getCameraOffsetMagnitude()!= 0.0),
                                    new CVIntakeCommand(robot)
                            )
                        ),
                        new TransferCommand(robot),


                        // Deposit inside sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new FollowPathChainCommand(robot.follower, paths.get(2)).disableUseIsBusy()
                                                .setCompletionThreshold(0.9)
                                )

                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot),


                        // Pickup middle sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),

                        new ParallelCommandGroup(
                                new FollowPathChainCommand(robot.follower, paths.get(3)).setHoldEnd(true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                                new HoverCommand(robot, 900)
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->robot.follower.getVelocityMagnitude() < CV_INTAKE_MAX_VELOCITY),
                                        new WaitUntilCommand(()->robot.sampleDetectionPipeline.getCameraOffsetMagnitude()!= 0.0),
                                        new CVIntakeCommand(robot)
                                )
                        ),


                        new TransferCommand(robot),

                        // Deposit middle sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new FollowPathChainCommand(robot.follower, paths.get(4)).disableUseIsBusy()
                                                .setCompletionThreshold(0.9)
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
                                                new HoverCommand(robot, 1000)
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->robot.follower.getVelocityMagnitude() < CV_INTAKE_MAX_VELOCITY),
                                        new WaitUntilCommand(()->robot.sampleDetectionPipeline.getCameraOffsetMagnitude()!= 0.0),
                                        new CVIntakeCommand(robot)
                                )
                        ),

//                        new InstantCommand(()->robot.intakeClawRotationServo.setPosition(0.45)),
//                        new IntakeSampleCommand(robot),
                        new TransferCommand(robot),

                        // Deposit outside sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new FollowPathChainCommand(robot.follower, paths.get(6)).disableUseIsBusy()
                                                .setCompletionThreshold(0.9)
                                )

                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot),

                        // sub cycle #1 pickup (5th samp)
                        new FollowPathChainCommand(robot.follower, paths.get(7)).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                                new WaitCommand(600),
                                                new HoverCommand(robot, (91.5 - (Globals.SampleAutonomousConfig.samp1Y + 72) - Globals.ROBOT_LENGTH/2 - Globals.INTAKE_MINIMUM_EXTENSION)*Globals.EXTENDO_TICKS_PER_INCH)
                                )
                        ),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(()->robot.follower.getVelocityMagnitude() < CV_INTAKE_MAX_VELOCITY),
                                new WaitUntilCommand(()->robot.sampleDetectionPipeline.getCameraOffsetMagnitude()!= 0.0),
                                new CVIntakeCommand(robot))
                        ),

                        // sub cycle #1 deposit (5th samp)
                        new FollowPathChainCommand(robot.follower, paths.get(8)).alongWith(
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        //new WaitCommand(0),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )
                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        // sub cycle #2 pickup (6th samp)
                        new FollowPathChainCommand(robot.follower, paths.get(9)).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED).alongWith(
                                                new WaitCommand(600),
                                                new HoverCommand(robot, (91.5 - (Globals.SampleAutonomousConfig.samp1Y + 72) - Globals.ROBOT_LENGTH/2 - Globals.INTAKE_MINIMUM_EXTENSION)*Globals.EXTENDO_TICKS_PER_INCH)
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->robot.follower.getVelocityMagnitude() < CV_INTAKE_MAX_VELOCITY),
                                        new WaitUntilCommand(()->robot.sampleDetectionPipeline.getCameraOffsetMagnitude()!= 0.0),
                                        new CVIntakeCommand(robot))
                        ),

                        // sub cycle #2 deposit (6th samp)
                        new FollowPathChainCommand(robot.follower, paths.get(10)).alongWith(
                                new SequentialCommandGroup(
                                        new TransferCommand(robot),
                                        //new WaitCommand(0),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET)
                                )
                        ),
                        new WaitCommand(150),
                        new DepositSampleCommand(robot)



//                        // Park
//                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
//                        new FollowPathChainCommand(robot.follower, paths.get(7)).setHoldEnd(false).alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new LiftCommand(robot, LiftSubsystem.LiftState.LVL1_ASCENT)
//                                )
//                        )



                )
            );
    }

    @Override
    public void run(){
        super.run();

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
        robot.telemetryA.addLine("eye of Sauron shutting down...");
        robot.setAutoCameraControls();
        robot.closeCamera();

    }
}