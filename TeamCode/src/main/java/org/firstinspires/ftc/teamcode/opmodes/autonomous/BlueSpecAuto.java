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
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
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
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator;

import java.util.ArrayList;

@Config
@Autonomous(name = "A🔵 Blue spec (4+0) Auto", group = "blue auto")
public class BlueSpecAuto extends CommandOpMode {
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
        robot.follower.setPose(allianceColor.convertPose(Globals.specAutoStartPose));


        SpecimenCycleGenerator specimenCyclePaths = new SpecimenCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        // specimen preload
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(6.465, 63.715, Point.CARTESIAN),
                                        new Point(38.913, 63.715, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setPathEndVelocityConstraint(3)
                        .build()
        );

        paths.add(specimenCyclePaths.getPickupPath());
        paths.add(specimenCyclePaths.getDepositPath());

        // bring back 2 samples
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierCurve(
                                        new Point(38.913, 63.715, Point.CARTESIAN),
                                        new Point(0.775, 22.869, Point.CARTESIAN),
                                        new Point(93.028, 40.118, Point.CARTESIAN),
                                        new Point(60.000, 23.900, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(60.000, 23.900, Point.CARTESIAN),
                                        new Point(16.314, 23.900, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(16.314, 23.900, Point.CARTESIAN),
                                        new Point(67.639, 27.327, Point.CARTESIAN),
                                        new Point(60.000, 13.800, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                        .addPath(
                                // Line 8
                                new BezierLine(
                                        new Point(60.000, 13.800, Point.CARTESIAN),
                                        new Point(16.314, 13.800, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(16.314, 13.800, Point.CARTESIAN),
                                        new Point(20.544, 32.754, Point.CARTESIAN),
                                        new Point(7.595, 31.342, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTValueConstraint(0.9)
                        .addParametricCallback(0.2, ()->robot.follower.setMaxPower(0.3))
                        .setPathEndVelocityConstraint(3)

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
                        // Deposit preload specimen
                        new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new DepositSpecimenCommand(robot),

                        new FollowPathCommand(robot.follower, paths.get(1)).setHoldEnd(true)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                                                new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                                new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                                ),
                        new WaitCommand(300),
                        new IntakeSpecimenCommand(robot)
//                        new FollowPathCommand(robot.follower, paths.get(2)).setHoldEnd(true),
//                        new WaitCommand(500),
//                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
//                        new FollowPathCommand(robot.follower, paths.get(3)).setHoldEnd(true)



                        // Pickup first specimen
                        //new InstantCommand(()-> robot.follower.setMaxPower(1))

            )
        );
        robot.reset();
        robot.lift.updateState(LiftSubsystem.LiftState.HOLDING_SPECIMEN);
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


        loopTime = loop;
    }
}