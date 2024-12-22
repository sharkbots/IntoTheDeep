package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.commandBase.AutoCommandOpMode;
import org.firstinspires.ftc.teamcode.common.utils.commandBase.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SampleCycleGenerator;


import java.util.ArrayList;
import java.util.concurrent.Callable;

@Config
@Autonomous(name = "🔵 Blue 4+0 Auto 2", group = "blue auto")
public class Blue4Plus0Auto2 extends CommandOpMode {
    private Telemetry telemetryA;

    private final Robot robot = Robot.getInstance();
    private Follower follower;

    private final Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePaths(){
        follower.setStartingPose(allianceColor.convertPose(Globals.preloadSampleStartPose));

        SampleCycleGenerator sampleCyclePaths = new SampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(follower);

        paths.add(
                follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Point(6.595, 101.105, Point.CARTESIAN),
                                new Point(12.386, 128.573, Point.CARTESIAN)
                        ))
                        //.addParametricCallback(0.5, () -> follower.setMaxPower(0.66))
                        //.setPathEndTValueConstraint(0.5) // faster end due to overshoot
                        //.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                        .build()
        );

        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.INSIDE));
        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.MIDDLE));
        paths.add(sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.OUTSIDE));

        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.INSIDE));
        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.MIDDLE));
        paths.add(sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.OUTSIDE));
    }
    @Override
    public void initialize() {
        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Globals.AllianceColor.BLUE;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        timer.reset();

        super.reset();

        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.44);

        generatePaths();

        schedule(
            new RunCommand(follower::update),
            new SequentialCommandGroup(
                    //new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET),
                    new FollowPathCommand(follower, paths.get(0)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(1)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(2)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(3)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(4)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(5)).setHoldEnd(false),
                    new WaitCommand(1500),
                    new FollowPathCommand(follower, paths.get(6)).setHoldEnd(false)

            )
        );

        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);
        Drawing.drawRobot(follower.poseUpdater.getPose(), "4CAF50");
        Drawing.sendPacket();

    }

    @Override
    public void run(){
        super.run();

        telemetryA.addData("Robot Pose", follower.getPose());
        telemetryA.update();

        dashboardPoseTracker.update();
        Drawing.drawRobot(follower.poseUpdater.getPose(), "4CAF50");
        Drawing.sendPacket();
    }
}