package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SampleCycleGenerator;


import java.util.ArrayList;

@Config
@Autonomous(name = "A🔵 Blue 4+0 Auto", group = "blue auto")
public class Blue4Plus0Auto extends CommandOpMode {
    private Telemetry telemetryA;

    private final Robot robot = Robot.getInstance();

    private final Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePaths(){
        //robot.follower.setStartingPose(allianceColor.convertPose(Globals.preloadSampleStartPose));
        robot.follower.setPose(allianceColor.convertPose(Globals.preloadSampleStartPose));


        SampleCycleGenerator sampleCyclePaths = new SampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(new BezierCurve(
                                allianceColor.convertPoint(new Point(6.595, 101.105, Point.CARTESIAN)),
                                allianceColor.convertPoint(new Point(22.486, 117.189, Point.CARTESIAN)),
                                allianceColor.convertPoint(new Point(12.386, 128.573, Point.CARTESIAN))
                        ))
                        .addParametricCallback(0.7, () -> robot.follower.setMaxPower(0.3))
                        //.setPathEndTValueConstraint(0.5) // faster end due to overshoot
                        //.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                        .setPathEndTValueConstraint(0.9)
                        .build()
        );

        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.INSIDE));
        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.INSIDE));


        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.MIDDLE));
        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.MIDDLE));

        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.OUTSIDE));
        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.OUTSIDE));

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        allianceColor.convertPoint(new Point(12.386, 128.573, Point.CARTESIAN)),
                                        allianceColor.convertPoint(new Point(41.514, 117.189, Point.CARTESIAN)),
                                        allianceColor.convertPoint(new Point(59.027, 122.162, Point.CARTESIAN))
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        allianceColor.convertPoint(new Point(59.027, 122.162, Point.CARTESIAN)),
                                        allianceColor.convertPoint(new Point(62.054, 91.527, Point.CARTESIAN))
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .addParametricCallback(0.9, ()-> robot.follower.setMaxPower(0.8))
                        .build()
        );
    }
    @Override
    public void initialize() {
        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Globals.AllianceColor.BLUE;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        timer.reset();

        super.reset();

        robot.init(hardwareMap);

        robot.follower.setMaxPower(0.7);

        generatePaths();

        schedule(
                new RunCommand(robot::clearChubCache),
                new RunCommand(robot::read),
                new RunCommand(robot::write),
                new RunCommand(robot::periodic),
                new RunCommand(robot.follower::update),

                new SequentialCommandGroup(
                        // Deposit preload sample
                        new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET)
                                )
                        ),
                        new WaitCommand(200),
                        new DepositClawCommand(robot, LiftSubsystem.ClawState.OPEN),

                        // Pickup inside sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new HoverCommand(robot, 1300),
                        new FollowPathCommand(robot.follower, paths.get(1)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        new InstantCommand(()->robot.intakeClawRotationServo.setPosition(0.59)),
                        new IntakeCommand(robot),

                        // Deposit inside sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        new FollowPathCommand(robot.follower, paths.get(2)).setHoldEnd(true)
                                )

                        ),
                        new WaitCommand(200),
                        new DepositClawCommand(robot, LiftSubsystem.ClawState.OPEN),

                        // Pickup middle sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new HoverCommand(robot, 1440),
                        new FollowPathCommand(robot.follower, paths.get(3)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        new IntakeCommand(robot),

                        // Deposit middle sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        new FollowPathCommand(robot.follower, paths.get(4)).setHoldEnd(true)
                                )

                        ),
                        new WaitCommand(200),
                        new DepositClawCommand(robot, LiftSubsystem.ClawState.OPEN),

                        // Pickup outside sample
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new HoverCommand(robot, 1550),
                        new FollowPathCommand(robot.follower, paths.get(5)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                                )
                        ),
                        new InstantCommand(()->robot.intakeClawRotationServo.setPosition(0.4)),
                        new IntakeCommand(robot),

                        // Deposit outside sample
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        new FollowPathCommand(robot.follower, paths.get(6)).setHoldEnd(true)
                                )

                        ),
                        new WaitCommand(200),
                        new DepositClawCommand(robot, LiftSubsystem.ClawState.OPEN),

                        // Park
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathCommand(robot.follower, paths.get(7)).setHoldEnd(false).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.LOW_HANG)
                                )
                        )
            )
        );
        robot.reset();
        robot.lift.updateState(LiftSubsystem.ClawState.CLOSED);


        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "4CAF50");
        Drawing.sendPacket();

    }

    @Override
    public void run(){
        super.run();

        telemetryA.addData("Robot Pose", robot.follower.getPose());
        double loop = System.nanoTime();
        telemetryA.addData("hz ", 1000000000 / (loop - loopTime));
        telemetryA.addLine(robot.follower.getPose().toString());
        telemetryA.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        telemetryA.addData("Lift pos", robot.liftActuator.getPosition());
        telemetryA.addData("Lift motor powers", robot.liftActuator.getPower());
        telemetryA.update();

        //dashboardPoseTracker.update();
        //Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "4CAF50");
        //Drawing.sendPacket();

        loopTime = loop;
    }
}