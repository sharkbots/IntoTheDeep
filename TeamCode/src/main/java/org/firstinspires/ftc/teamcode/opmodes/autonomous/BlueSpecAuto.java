package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.specAutoStartPose;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.depositLocation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator;

import java.util.ArrayList;

@Config
@Autonomous(name = "A🔵 Blue spec (4+0) Auto", group = "blue auto", preselectTeleOp = "Two Driver Teleop")
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
        //robot.follower.setStartingPose(allianceColor.convert(Globals.preloadSampleStartPose));
        robot.follower.setPose(allianceColor.convert(specAutoStartPose, Pose.class));


        SpecimenCycleGenerator specimenCyclePaths = new SpecimenCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        // deposit specimen 1
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        allianceColor.convert(specAutoStartPose, Point.class),
                                        allianceColor.convert(depositLocation, Point.class)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setPathEndVelocityConstraint(3)
                        .setPathEndTimeoutConstraint(250)
                        .addParametricCallback(0.7, ()-> robot.follower.setMaxPower(0.3))
                        //.setPathEndTValueConstraint(0.95)
                        .build()
        ); // path 0

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierCurve(
                                        new Point(37.5, 63.72, Point.CARTESIAN),
                                        new Point(22.054, 27.027, Point.CARTESIAN),
                                        new Point(71.351, 41.081, Point.CARTESIAN),
                                        new Point(57.730, 22.919, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(57.730, 22.919, Point.CARTESIAN),
                                        new Point(16.314, 23.900, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(16.314, 23.900, Point.CARTESIAN),
                                        new Point(67.639, 27.327, Point.CARTESIAN),
                                        new Point(60.000, 13.800, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(60.000, 13.800, Point.CARTESIAN),
                                        new Point(16.314, 13.800, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addParametricCallback(0.7, ()-> robot.follower.setMaxPower(0.7))
                        .setZeroPowerAccelerationMultiplier(2)
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(16.314, 13.800, Point.CARTESIAN),
                                        new Point(16.314, 30, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setZeroPowerAccelerationMultiplier(2)
                        .addParametricCallback(0.9, ()-> robot.follower.setMaxPower(0.3))
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(16.314, 30, Point.CARTESIAN),
                                        new Point(7.595, 32.342, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        ); // path 1

        // deposit specimen 2
        paths.add(specimenCyclePaths.getDepositPath(1)); // path 2


        // deposit specimen 3
        paths.add(specimenCyclePaths.getPickupPath(2)); // path 3
        paths.add(specimenCyclePaths.getDepositPath(2)); // path 4

        // specimen 4
        paths.add(specimenCyclePaths.getPickupPath(3)); // path 5
        paths.add(specimenCyclePaths.getDepositPath(3)); // path 6

        // park
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(depositLocation.getX(), depositLocation.getY()-(3-1)*1.5),
                                        new Point(12, 24)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        ); // path 7
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
                        // Deposit specimen 1 (preload)
                        new FollowPathCommand(robot.follower, paths.get(0)).setHoldEnd(false).setCompletionThreshold(0.95).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)

                                )
                        ),
                        new InstantCommand(()-> robot.follower.setMaxPower(0.7)),


                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_DOWN),
                        new DepositSpecimenCommand(robot),

                        // bring two specimens back && pickup specimen 2
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathCommand(robot.follower, paths.get(1)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true)
                                )
                        ),
                        new WaitCommand(100),
                        // pickup specimen 2
                        new IntakeSpecimenCommand(robot),

                        new InstantCommand(()-> robot.follower.setMaxPower(0.7)),
                        // Deposit specimen 2
                        new FollowPathCommand(robot.follower, paths.get(2)).setHoldEnd(false).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_DOWN),
                        new DepositSpecimenCommand(robot),

                        // Pickup specimen 3
                        new FollowPathCommand(robot.follower, paths.get(3)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                        ),
                        new WaitCommand(100),
                        new IntakeSpecimenCommand(robot),

                        // Deposit specimen 3
                        new FollowPathCommand(robot.follower, paths.get(4)).setHoldEnd(false).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_DOWN),
                        new DepositSpecimenCommand(robot),

                        // Pickup specimen 4
                        new FollowPathCommand(robot.follower, paths.get(5)).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                        ),
                        new WaitCommand(100),
                        new IntakeSpecimenCommand(robot),

                        // Deposit specimen 4
                        new FollowPathCommand(robot.follower, paths.get(6)).setHoldEnd(false).setCompletionThreshold(0.995).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_DOWN),
                        new DepositSpecimenCommand(robot),

                        // park
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathCommand(robot.follower, paths.get(7)).setHoldEnd(true).alongWith(
                                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
                        )
                )
        );
        robot.reset();
        robot.lift.updateState(LiftSubsystem.LiftState.HOLDING_SPECIMEN);
        robot.lift.updateState(LiftSubsystem.ClawState.CLOSED);
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
        Globals.END_OF_AUTO_POSE = robot.follower.poseUpdater.getPose();
    }
}