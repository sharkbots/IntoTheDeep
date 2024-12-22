package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.commandBase.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SampleCycleGenerator;

import java.util.concurrent.Callable;

@Config
@Autonomous(name = "🔵 Blue 4+0 Auto", group = "blue auto")
public class Blue4Plus0Auto extends LinearOpMode {
    private final Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    //private final Pose preloadSampleStartPose = new Pose(6.595,101.105, Math.toRadians(270));

    private final Robot robot = Robot.getInstance();
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    Follower follower;


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Globals.AllianceColor.BLUE;

//        robot.init(hardwareMap);
//        robot.addSubsystem(robot.intake);
//        robot.addSubsystem(robot.lift);
//        robot.addSubsystem(robot.drivetrain);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(Globals.preloadSampleStartPose);


        SampleCycleGenerator sampleCyclePaths = new SampleCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(follower);

        PathChain preloadSampleDeposit = new PathBuilder()
                .addPath(new BezierLine(
                        new Point(6.595,101.105, Point.CARTESIAN),
                        new Point(12.386, 101.105, Point.CARTESIAN)
                ))
                //.addParametricCallback(0.5, () -> follower.setMaxPower(0.66))
                //.setPathEndTValueConstraint(0.5) // faster end due to overshoot
                //.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        PathChain insideSamplePickup = sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.INSIDE);
        PathChain middleSamplePickup = sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.MIDDLE);
        PathChain outsideSamplePickup = sampleCyclePaths.getSamplePath(SampleCycleGenerator.SampleLocation.OUTSIDE);

        PathChain insideSampleDeposit = sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.INSIDE);
        PathChain middleSampleDeposit = sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.MIDDLE);
        PathChain outsideSampleDeposit = sampleCyclePaths.getBucketPath(SampleCycleGenerator.SampleLocation.OUTSIDE);



        //robot.read();

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new WaitCommand(1500),
                        new FollowPathCommand(follower, preloadSampleDeposit)//,
                        //new WaitCommand(1000),
//                        new FollowPathCommand(follower, insideSamplePickup),
//                        new FollowPathCommand(follower, insideSampleDeposit),
//                        new FollowPathCommand(follower, middleSamplePickup),
//                        new FollowPathCommand(follower, middleSampleDeposit),
//                        new FollowPathCommand(follower, outsideSamplePickup),
//                        new FollowPathCommand(follower, outsideSampleDeposit)
//                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET),
//                        new WaitCommand(1500),
//                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
//                        new WaitCommand(1500),
//                        new HoverCommand(robot, 800)
                        //new FollowPathCommand(follower, insideSamplePickup),
                        //new WaitCommand(500),
                        //new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_BASKET)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            //robot.clearBulkCache();
//            robot.read();
//            robot.periodic();
//            robot.write();
            follower.update();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(follower.getPose().toString());
            telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
            telemetry.update();

            loopTime = loop;
        }

        Globals.END_OF_AUTO_POSE = follower.getPose();
        robot.kill();
    }
}