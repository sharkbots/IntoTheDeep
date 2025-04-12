package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.specAutoStartPose;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.depositLocation;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.intermediatePickupLocation;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.pickupLocation;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.CVIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.SetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator;

import java.util.ArrayList;

@Config
@Autonomous(name = "🔵/🔴 6-Spec Auto", group = "1 blue auto", preselectTeleOp = "Two Driver Teleop")
public class SixSpecAuto extends CommandOpMode {
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
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        //.setZeroPowerAccelerationMultiplier(3)
                        //.setPathEndVelocityConstraint(3)
                        //.setPathEndTimeoutConstraint(250)
                        //.addParametricCallback(0.7, ()-> robot.follower.setMaxPower(0.3))
                        //.setPathEndTValueConstraint(0.95)
                        .build()
        ); // path 0

        paths.add(specimenCyclePaths.getDepositPathStrafe(1)); // path 1

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
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(56.216, 31.100, Point.CARTESIAN),
                                        new Point(51.946, 19.243, Point.CARTESIAN),
                                        new Point(25.314, 21.400, Point.CARTESIAN)
                                )
                        )
//                        .setZeroPowerAccelerationMultiplier(2)
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
//                        .setZeroPowerAccelerationMultiplier(2)
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
                                // Line 8
                                new BezierLine(
                                        new Point(49.216, 7.300, Point.CARTESIAN),
                                        new Point(23.314, 7.300, Point.CARTESIAN)
                                )
                        )
//                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 9
                                new BezierLine(
                                        new Point(23.314, 7.300, Point.CARTESIAN),
                                        new Point(pickupLocation.getX()+2.5, 7.300+5, Point.CARTESIAN)
                                )
                        )
                        .addParametricCallback(0.7, ()->robot.follower.setMaxPower(0.6))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
//                        .addPath(
//                                // Line 10
//                                new BezierLine(
//                                        new Point(17.297, 29.342, Point.CARTESIAN),
//                                        new Point(7.595, 32.342, Point.CARTESIAN)
//                                )
//                        )
//                        .setConstantHeadingInterpolation(Math.toRadians(0))
//                        .addParametricCallback(0.0, ()-> robot.follower.setMaxPower(0.7))
                        .build()
        ); // path 2


        // pickup spec 4
        paths.add(specimenCyclePaths.getPickupPathSpline(3));

        // depo spec 4
        paths.add(specimenCyclePaths.getDepositPathSpline(3));

        // pickup spec 5
        paths.add(specimenCyclePaths.getPickupPathSpline(4));

        // depo spec 5
        paths.add(specimenCyclePaths.getDepositPathSpline(4));

        // pickup spec 6
        paths.add(specimenCyclePaths.getPickupPathSpline(5));

        // depo spec 6
        paths.add(specimenCyclePaths.getDepositPathSpline(5));


//        // pickup spec 4
//        paths.add(specimenCyclePaths.getPickupPathStrafe(3));
//
//        // depo spec 4
//        paths.add(specimenCyclePaths.getDepositPathStrafe(3));
//
//        // pickup spec 5
//        paths.add(specimenCyclePaths.getPickupPathStrafe(4));
//
//        // depo spec 5
//        paths.add(specimenCyclePaths.getDepositPathStrafe(4));
//
//        // pickup spec 6
//        paths.add(specimenCyclePaths.getPickupPathStrafe(5));
//
//        // depo spec 6
//        paths.add(specimenCyclePaths.getDepositPathStrafe(5));



//
//        // park
//        paths.add(
//
//                robot.follower.pathBuilder()
//                        .addPath(
//                                // Line 13
//                                new BezierCurve(
//                                        new Point(40.432, 61.720, Point.CARTESIAN),
//                                        new Point(36.4, 61.72, Point.CARTESIAN),
//                                        new Point(19.459, 42.811, Point.CARTESIAN)
//                                )
//                        )
//                        .setTangentHeadingInterpolation()
//                        .build()
//        ); // path 9
    }

    public SequentialCommandGroup specimenCycle (int cycleNum) {
        return new SequentialCommandGroup();
    }
    @Override
    public void initialize() {
        super.reset();
        Globals.IS_AUTONOMOUS = true;
        Globals.GRABBING_MODES.set(Globals.GRABBING_MODES.SPECIMEN);
        //Globals.ALLIANCE_FIXED_VAL = Globals.AllianceColor.BLUE;

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
//                        // Deposit specimen 1 (preload)
//                        new FollowPathChainCommand(robot.follower, paths.get(0)).setHoldEnd(false),
//                        new DeferredCommand(()->
//                                // Pickup second spec
//                                new FollowPathChainCommand(robot.follower,
//                                        robot.follower.pathBuilder().addPath(
//                                                        new BezierLine(
//                                                                robot.follower.getPose(),
//                                                                allianceColor.convert(pickupLocation, Pose.class)
//                                                        )
//                                                )
//                                                .setPathEndTValueConstraint(0.995)
//                                                .setConstantHeadingInterpolation(Math.toRadians(0))
//                                                .build()
//                                )
//                                , null),
//
//                        // deposit spec 2
//                        new FollowPathChainCommand(robot.follower, paths.get(1)),
//
//                        // spec 3
//                        // Push all 3 into sub
//                        new FollowPathChainCommand(robot.follower, paths.get(2)),
//
//                        // Deposit spec 3
//                        new DeferredCommand(()->
//                                new FollowPathChainCommand(robot.follower,
//                                        robot.follower.pathBuilder().addPath(
//                                                        new BezierLine(
//                                                                robot.follower.getPose(),
//                                                                new Pose(depositLocation.getX(), depositLocation.getY()-2*1.5)
//                                                        )
//                                                ).setConstantHeadingInterpolation(Math.toRadians(0))
//                                                .build()
//                                )
//                                , null),
//
//                        // spec 4
//                        new FollowPathChainCommand(robot.follower, paths.get(3)),
//                        new FollowPathChainCommand(robot.follower, paths.get(4)),
//                        new FollowPathChainCommand(robot.follower, paths.get(5)),
//                        new FollowPathChainCommand(robot.follower, paths.get(6))


                        // Deposit specimen 1 (preload)
                        new FollowPathChainCommand(robot.follower, paths.get(0)).setHoldEnd(false)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)

                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN).alongWith(new HoverCommand(robot, 300)),

                        // Intake sample from sub
                        new DepositSpecimenCommand(robot).andThen(
                                new ParallelCommandGroup(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.TRANSFER),
                                        new CVIntakeCommand(robot, "blue")
                                )
                        ),
                        // pickup spec 2
                        new TransferCommand(robot),
                        new ParallelCommandGroup(
                                new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN).andThen(
                                        new WaitCommand(1000).alongWith(
                                                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0)
                                        ),
                                        new InstantCommand(()->robot.lift.setClawState(LiftSubsystem.ClawState.OPEN))
                                ),
                                new DeferredCommand(()->
                                        // Pickup second spec
                                        new FollowPathChainCommand(robot.follower,
                                                robot.follower.pathBuilder().addPath(
                                                                new BezierCurve(
                                                                        robot.follower.getPose(),
                                                                        new Pose(30.44,  robot.follower.getPose().getY()),
                                                                        new Pose(40.3, pickupLocation.getY()),
                                                                        allianceColor.convert(pickupLocation, Pose.class)
                                                                )
                                                        )
                                                        .setPathEndTValueConstraint(0.95)
                                                        .setConstantHeadingInterpolation(Math.toRadians(0))
                                                        .setZeroPowerAccelerationMultiplier(4)
                                                        .build()
                                        )
                                        , null)
                        ),
                        new IntakeSpecimenCommand(robot),

                        // deposit spec 2
                        new FollowPathChainCommand(robot.follower, paths.get(1))
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                        )
                                ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),


                        // spec 3
                        // Push all 3 into sub
                        new FollowPathChainCommand(robot.follower, paths.get(2))
                                .alongWith(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
                                ),
                        new IntakeSpecimenCommand(robot),

                        // Deposit spec 3
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new DeferredCommand(()->
                                new FollowPathChainCommand(robot.follower,
                                        robot.follower.pathBuilder().addPath(
                                                new BezierCurve(
                                                        robot.follower.getPose(),
                                                        new Pose(depositLocation.getX()-5, depositLocation.getY()-2*1.5),
                                                        new Pose(depositLocation.getX()-5, depositLocation.getY()-2*1.5),
                                                        new Pose(depositLocation.getX(), depositLocation.getY()-2*1.5))
                                        ).setConstantHeadingInterpolation(Math.toRadians(0))
                                                .build()
                                )
                                , null)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // spec 4
                        new FollowPathChainCommand(robot.follower, paths.get(3))
                                .alongWith(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
                                ),
                        new IntakeSpecimenCommand(robot),

                        new FollowPathChainCommand(robot.follower, paths.get(4))
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                        )
                                ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // spec 5
                        new FollowPathChainCommand(robot.follower, paths.get(5))
                                .alongWith(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
                                ),
                        new IntakeSpecimenCommand(robot),

                        new FollowPathChainCommand(robot.follower, paths.get(6))
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                        )
                                ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot),

                        // spec 6
                        new FollowPathChainCommand(robot.follower, paths.get(7))
                                .alongWith(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)
                                ),
                        new IntakeSpecimenCommand(robot),

                        new FollowPathChainCommand(robot.follower, paths.get(8))
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                        )
                                ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot)



                )

        );
        robot.intake.setExtendoTargetTicks(0);
        robot.intake.setPivotState(IntakeSubsystem.PivotState.FULLY_RETRACTED);
        robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN);
        robot.intake.setClawRotationDegrees(0);
        //robot.reset();
        robot.lift.updateState(LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP);
        robot.lift.setLiftTargetPosTicks(150);
        robot.lift.setClawState(LiftSubsystem.ClawState.CLOSED);


    }

    @Override
    public void run(){
        super.run();

//        robot.telemetryA.addData("Robot Pose", robot.follower.getPose());
        double loop = System.nanoTime();
        robot.telemetryA.addData("feedforward", robot.liftActuator.getCurrentFeedforward());
//        robot.telemetryA.addData("hz ", 1000000000 / (loop - loopTime));
//        robot.telemetryA.addLine(robot.follower.getPose().toString());
//        robot.telemetryA.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        robot.telemetryA.addData("Lift pos", robot.liftActuator.getPosition());
        robot.telemetryA.addData("Lift target", robot.liftActuator.getTargetPosition());
        robot.telemetryA.addData("Lift motor powers", robot.liftActuator.getPower());
//        robot.telemetryA.addData("t value (general loop)", robot.follower.getCurrentTValue());
        if (robot.vision.limelight.getLatestResults() != null) {
            robot.telemetryA.addLine("detection is not null");
            LLResultTypes.DetectorResult result = robot.vision.limelight.getClosestResult();
            float[] offsets = robot.vision.limelight.getClosestOffset();
            if (result != null) {
                robot.telemetryA.addData("Closest result (Pixels): ", result.getTargetXPixels() + ", " + result.getTargetYPixels());
                robot.telemetryA.addData("Closest result (Offset): ", offsets[0] + ", " + offsets[1] + ", " + offsets[2]);
                robot.telemetryA.addData("Closest result color: ", result.getClassName());
            }
        }

        robot.telemetryA.update();

        loopTime = loop;
        Globals.END_OF_AUTO_POSE = robot.follower.poseUpdater.getPose();
    }
}