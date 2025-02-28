package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.specAutoStartPose;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.depositLocation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator;

import java.util.ArrayList;

@Config
@Autonomous(name = "A🔵 Blue spec (5+0) Auto", group = "blue auto", preselectTeleOp = "Two Driver Teleop")
public class FiveSpecAuto extends CommandOpMode {
    //private Telemetry telemetryA;

    private final Robot robot = Robot.getInstance();

    private final Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    public GamepadEx operator;
    ConfigMenu menu;

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
                                // Line 2
                                new BezierCurve(
                                        new Point(37.500, 63.720, Point.CARTESIAN),
                                        new Point(27.676, 45.622, Point.CARTESIAN),
                                        new Point(44.757, 31.100, Point.CARTESIAN),
                                        new Point(56.216, 31.100, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(56.216, 31.100, Point.CARTESIAN),
                                        new Point(51.946, 19.243, Point.CARTESIAN),
                                        new Point(25.314, 21.400, Point.CARTESIAN)
                                )
                        )
                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(25.314, 21.400, Point.CARTESIAN),
                                        new Point(50.216, 21.100, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 5
                                new BezierCurve(
                                        new Point(50.216, 21.100, Point.CARTESIAN),
                                        new Point(51.946, 6.243, Point.CARTESIAN),
                                        new Point(25.314, 11.400, Point.CARTESIAN)
                                )
                        )
                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(25.314, 11.400, Point.CARTESIAN),
                                        new Point(49.216, 10.811, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(49.216, 10.811, Point.CARTESIAN),
                                        new Point(49.216, 7.300, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 8\



                                new BezierLine(
                                        new Point(49.216, 7.300, Point.CARTESIAN),
                                        new Point(23.314, 7.300, Point.CARTESIAN)
                                )
                        )
                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 9
                                new BezierLine(
                                        new Point(23.314, 7.300, Point.CARTESIAN),
                                        new Point(17.297, 29.342, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 10
                                new BezierLine(
                                        new Point(17.297, 29.342, Point.CARTESIAN),
                                        new Point(7.595, 32.342, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addParametricCallback(0.0, ()-> robot.follower.setMaxPower(1))
                        .build()
        );


        // deposit specimen 2
        paths.add(specimenCyclePaths.getDepositPath(1)); // path 2

        // pickup spec 3
        paths.add(specimenCyclePaths.getPickupPath(2)); // path 3

        // depo spec 3
        paths.add(specimenCyclePaths.getDepositPath(2)); // path 4

        // pickup spec 4
        paths.add(specimenCyclePaths.getPickupPath(3)); // path 5

        // depo spec 4
        paths.add(specimenCyclePaths.getDepositPath(3)); // path 6

        // pickup spec 4
        paths.add(specimenCyclePaths.getPickupPath(4)); // path 7

        // depo spec 4
        paths.add(specimenCyclePaths.getDepositPath(4)); // path 8

        // park
        paths.add(

                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(40.432, 61.720, Point.CARTESIAN),
                                        new Point(36.4, 61.72, Point.CARTESIAN),
                                        new Point(19.459, 42.811, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        ); // path 9
    }

    public SequentialCommandGroup specimenCycle (int cycleNum) {
        return new SequentialCommandGroup();
    }
    @Override
    public void initialize() {
        super.reset();
        Globals.IS_AUTONOMOUS = true;
        Globals.ALLIANCE = Globals.AllianceColor.BLUE;

        operator = new GamepadEx(gamepad2);

        robot.setTelemetry(telemetry);

        timer.reset();

        super.reset();

        robot.init(hardwareMap);

        robot.follower.setMaxPower(1); //0.7

        generatePaths();

        schedule(
                new RunCommand(robot::clearChubCache),
                new RunCommand(robot::read),
                new RunCommand(robot::periodic),
                new RunCommand(robot::write),
                new RunCommand(robot.follower::update),

                new SequentialCommandGroup(
                        // Deposit specimen 1 (preload)
                        new FollowPathChainCommand(robot.follower, paths.get(0)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.95)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)

                                )
                        ),
                        new InstantCommand(()-> robot.follower.setMaxPower(1)), //0.9


                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // bring two specimens back && pickup specimen 2
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new FollowPathChainCommand(robot.follower, paths.get(1)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.99)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true)
                                )
                        ),
                        new IntakeSpecimenCommand(robot),

                        // Deposit specimen 2
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),

                        new FollowPathChainCommand(robot.follower, paths.get(2)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.99)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),


                        // Pickup specimen 3
                        new FollowPathChainCommand(robot.follower, paths.get(3)).setHoldEnd(false).disableUseIsBusy()
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                        ),
                        new IntakeSpecimenCommand(robot),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        // Deposit specimen 3
                        new FollowPathChainCommand(robot.follower, paths.get(4)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.99)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),


                        // Pickup specimen 4
                        new FollowPathChainCommand(robot.follower, paths.get(5)).setHoldEnd(false).disableUseIsBusy()
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                        ),
                        new IntakeSpecimenCommand(robot),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        // Deposit specimen 4
                        new FollowPathChainCommand(robot.follower, paths.get(6)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.99)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // Pickup specimen 5
                        new FollowPathChainCommand(robot.follower, paths.get(7)).setHoldEnd(false).disableUseIsBusy().setCompletionThreshold(0.99)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                        new InstantCommand(() -> Globals.INTAKING_SPECIMENS = true))
                        ),
                        new IntakeSpecimenCommand(robot),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        // Deposit specimen 5
                        new FollowPathChainCommand(robot.follower, paths.get(8)).disableUseIsBusy().setHoldEnd(false).setCompletionThreshold(0.97)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // Park
                        new FollowPathChainCommand(robot.follower, paths.get(9))
                                .alongWith(
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new WaitCommand(300),
                                                        new LiftCommand(robot, LiftSubsystem.LiftState.TRANSFER)
                                                ),
                                                new HoverCommand(robot, 1500)
                                        )
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

//        robot.telemetryA.addData("Robot Pose", robot.follower.getPose());
        double loop = System.nanoTime();
//        robot.telemetryA.addData("feedforward", robot.liftActuator.getCurrentFeedforward());
//        robot.telemetryA.addData("hz ", 1000000000 / (loop - loopTime));
//        robot.telemetryA.addLine(robot.follower.getPose().toString());
//        robot.telemetryA.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
//        robot.telemetryA.addData("Lift pos", robot.liftActuator.getPosition());
//        robot.telemetryA.addData("Lift target", robot.liftActuator.getTargetPosition());
//        robot.telemetryA.addData("Lift motor powers", robot.liftActuator.getPower());
//        robot.telemetryA.addData("t value (general loop)", robot.follower.getCurrentTValue());

        robot.telemetryA.update();

        loopTime = loop;
        Globals.END_OF_AUTO_POSE = robot.follower.poseUpdater.getPose();
    }
}