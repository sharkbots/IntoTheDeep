package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.depositLocation;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets.SpecimenCycleGenerator.pickupLocation;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.follower.FollowerConstants;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.FollowPathChainCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.CVIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.HoverCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.SetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.DepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.IntakeSpecimenAutoCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.vision.Sample;
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
    boolean alreadyCompiled = false;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;

    public SequentialCommandGroup specimenCycle (int cycleNum) {
        return new SequentialCommandGroup();
    }
    @Override
    public void initialize() {
        super.reset();
        Globals.IS_AUTONOMOUS = true;
        Globals.GRABBING_MODES.set(Globals.GRABBING_MODES.SPECIMEN);

        timer.reset();

        operator = new GamepadEx(gamepad2);

        robot.setTelemetry(telemetry);
        robot.telemetryA.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);

        menu = new ConfigMenu(operator, robot);
        menu.setConfigurationObject(new Globals.SpecAutonomousConfig());
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(menu::backupCurrentField);

        robot.init(hardwareMap);

        robot.follower.setMaxPower(1);

        robot.reset();

        robot.intake.setExtendoTargetTicks(0);
        robot.intake.setPivotState(IntakeSubsystem.PivotState.FULLY_RETRACTED);
        robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN);
        robot.intake.setClawRotationDegrees(0);
        //robot.reset();
        robot.lift.updateState(LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP);
        robot.lift.setLiftTargetPosTicks(150);
        robot.lift.setClawState(LiftSubsystem.ClawState.CLOSED);

        while(opModeInInit()){
            menu.periodic();

            if (menu.isLocked() && !alreadyCompiled){
                alreadyCompiled = true;

                Globals.ALLIANCE_COLOR = Globals.SpecAutonomousConfig.allianceColor;

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
        //robot.follower.setStartingPose(allianceColor.convert(Globals.preloadSampleStartPose));
        robot.follower.setPose(allianceColor.convert(Globals.specAutoStartPose, Pose.class));


        SpecimenCycleGenerator specimenCyclePaths = new SpecimenCycleGenerator()
                .setAlliance(allianceColor)
                .setFollower(robot.follower);

        // deposit specimen 1
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        allianceColor.convert(Globals.specAutoStartPose, Point.class),
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
                                        new Point(51.216, 31.100, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(51.216, 31.100, Point.CARTESIAN),
                                        new Point(51.946, 15.243+4, Point.CARTESIAN),
                                        new Point(25.314, 21.400, Point.CARTESIAN)
                                )
                        )
//                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(25.314, 21.400, Point.CARTESIAN),
                                        new Point(50.216-3-7, 21.100, Point.CARTESIAN)
                                )
                        )
                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addPath(
                                // Line 5
                                new BezierCurve(
                                        new Point(50.216-3-7, 21.100, Point.CARTESIAN),
                                        new Point(68.108, 3.243+2+2+2, Point.CARTESIAN),
                                        //new Point(55.946, 5.243, Point.CARTESIAN),
                                        new Point(23.314, 11.400, Point.CARTESIAN)
                                )
                        )
//                        .setZeroPowerAccelerationMultiplier(2)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
//                        .addPath(
//                                // Line 6
//                                new BezierLine(
//                                        new Point(23.314, 11.400, Point.CARTESIAN),
//                                        new Point(49.216-2, 10.811, Point.CARTESIAN)
//                                )
//                        )
//                        .setZeroPowerAccelerationMultiplier(2)
//                        .setConstantHeadingInterpolation(Math.toRadians(0))
//                        .addPath(
//                                // Line 7
//                                new BezierLine(
//                                        new Point(49.216-2, 10.811, Point.CARTESIAN),
//                                        new Point(52+3, 6.300, Point.CARTESIAN)
//                                )
//                        )
//                        .setPathEndTValueConstraint(0.98)
//                        .setConstantHeadingInterpolation(Math.toRadians(0))
//
//                        .addPath(
//                                new BezierCurve(
//                                        new Point(52+3, 6.300, Point.CARTESIAN),
//                                        new Point(14.18-5-2, 5.61),
//                                        new Point(26, 12),
//                                        new Point(pickupLocation.getX(), 12)
//                                )
//                        )
                        .addPath(
                                // Line 1
                                new BezierCurve(
                                        new Point(23.100, 11.400, Point.CARTESIAN),
                                        new Point(55.568, 13.622+3, Point.CARTESIAN),
                                        new Point(55.000-5, 6.300, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setZeroPowerAccelerationMultiplier(2)
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(55-5, 6.3, Point.CARTESIAN),
                                        new Point(7.180, 7.800, Point.CARTESIAN),
                                        new Point(33.000, 12.000, Point.CARTESIAN),
                                        new Point(pickupLocation.getX(), 12.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTValueConstraint(0.97)
                        //.addParametricCallback(0.7, ()-> robot.follower.setMaxPower(0.7))
                        //.setPathEndVelocityConstraint(3)
                        .setZeroPowerAccelerationMultiplier(3.5)
//                        .addPath(
//                                // Line 8
//                                new BezierLine(
//                                        new Point(49.216, 7.300, Point.CARTESIAN),
//                                        new Point(23.314, 7.300, Point.CARTESIAN)
//                                )
//                        )
////                        .setZeroPowerAccelerationMultiplier(2)
//                        .setConstantHeadingInterpolation(Math.toRadians(0))
//                        .addPath(
//                                // Line 9
//                                new BezierLine(
//                                        new Point(23.314, 7.300, Point.CARTESIAN),
//                                        new Point(pickupLocation.getX()+2.5, 7.300+5, Point.CARTESIAN)
//                                )
//                        )
//                        .addParametricCallback(0.7, ()->robot.follower.setMaxPower(0.6))
//                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        ); // path 2


        // pickup spec 4
        paths.add(specimenCyclePaths.getPickupPathSpline(3));

        // depo spec 4
        paths.add(specimenCyclePaths.getDepositPathStrafe(3));

        // pickup spec 5
        paths.add(specimenCyclePaths.getPickupPathSpline(4));

        // depo spec 5
        paths.add(specimenCyclePaths.getDepositPathStrafe(4));

        // pickup spec 6
        paths.add(specimenCyclePaths.getPickupPathSpline(5));

        // depo spec 6
        paths.add(specimenCyclePaths.getDepositPathStrafe(5));


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

    private void generateSchedule() {
        schedule(
                new RunCommand(robot::clearChubCache),
                new RunCommand(robot::read),
                new RunCommand(robot::periodic),
                new RunCommand(robot::write),
                new RunCommand(robot.follower::update),

                new SequentialCommandGroup(
                        // Deposit specimen 1 (preload)
                        new FollowPathChainCommand(robot.follower, paths.get(0)).setHoldEnd(false)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(200).alongWith(
                                                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0)
                                        ),
                                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)

                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),

                        // Intake sample from sub
                        new DepositSpecimenCommand(robot).andThen(
                                new WaitCommand(300),
                                new ParallelCommandGroup(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
//                                        new HoverCommand(robot, 300).andThen(
//                                                new IntakeSampleCommand(robot)
//                                        )
                                        new CVIntakeCommand(robot, Globals.ALLIANCE_COLOR==Globals.AllianceColor.BLUE? Sample.Color.BLUE: Sample.Color.RED)
                                )
                        ),
                        new ParallelCommandGroup(
                                new TransferCommand(robot).andThen(
                                        new ParallelCommandGroup(
                                                new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN).alongWith(
                                                        new InstantCommand(()->robot.depositClawRotationServo.setPosition(0.84))
                                                ),
                                                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),
                                                        new InstantCommand(()->robot.lift.setClawState(LiftSubsystem.ClawState.OPEN))

                                                )
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->robot.intake.getExtendoPosTicks() < 900),
                                        new WaitCommand(100),
                                        new DeferredCommand(()->
                                        // Pickup second spec
                                        new FollowPathChainCommand(robot.follower,
                                                robot.follower.pathBuilder().addPath(
                                                                new BezierCurve(
                                                                        robot.follower.getPose(),
                                                                        new Pose(30.44,  robot.follower.getPose().getY()),
                                                                        new Pose(45.3, pickupLocation.getY()),
                                                                        allianceColor.convert(pickupLocation, Pose.class)
                                                                )
                                                        )
                                                        .setPathEndTValueConstraint(0.95)
                                                        .setConstantHeadingInterpolation(Math.toRadians(0))
                                                        //.setPathEndVelocityConstraint(3)
                                                        .setPathEndHeadingConstraint(Math.toRadians(3))
                                                        .setZeroPowerAccelerationMultiplier(3.5)
                                                        //.addParametricCallback(0.8, ()-> robot.follower.setMaxPower(0.7))
                                                        .build()
                                        ), null))

                        ),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),

                        new IntakeSpecimenAutoCommand(robot).alongWith(
                                // Deposit spec 2
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(1))
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                                        )
                                                )
                                )
                        ),

//                      Old deposit spec 2 (without picking up sample)
                        new ParallelCommandGroup(
                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                                // spec 5
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(2))
                                                .alongWith(
                                                        new WaitCommand(200).andThen(new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)),
                                                        new WaitCommand(50).andThen(new DepositSpecimenCommand(robot))
                                                )
                                )
                        ),
                        // TODO: make this async with moving away?
                        new IntakeSpecimenAutoCommand(robot),

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



                        // new spec 3 deposit + samp 2 pickup
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),

                        // Intake sample from sub
                        new DepositSpecimenCommand(robot).andThen(
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED),
//                                        new HoverCommand(robot, 300).andThen(
//                                                new IntakeSampleCommand(robot)
//                                        )
                                        new CVIntakeCommand(robot, Globals.ALLIANCE_COLOR==Globals.AllianceColor.BLUE? Sample.Color.BLUE: Sample.Color.RED)
                                )
                        ),
                        new ParallelCommandGroup(
                                new TransferCommand(robot).andThen(
                                        new ParallelCommandGroup(
                                                new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN).alongWith(
                                                        new InstantCommand(()->robot.depositClawRotationServo.setPosition(0.84))
                                                ),
                                                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.FULLY_RETRACTED, 0.0),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),
                                                        new InstantCommand(()->robot.lift.setClawState(LiftSubsystem.ClawState.OPEN))

                                                )
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->robot.intake.getExtendoPosTicks() < 900),
                                        new WaitCommand(100),
                                        new DeferredCommand(()->
                                                // Pickup second spec
                                                new FollowPathChainCommand(robot.follower,
                                                        robot.follower.pathBuilder().addPath(
                                                                        new BezierCurve(
                                                                                robot.follower.getPose(),
                                                                                new Pose(30.44,  robot.follower.getPose().getY()),
                                                                                new Pose(45.3, pickupLocation.getY()),
                                                                                allianceColor.convert(pickupLocation, Pose.class)
                                                                        )
                                                                )
                                                                .setPathEndTValueConstraint(0.95)
                                                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                                                //.setPathEndVelocityConstraint(3)
                                                                .setPathEndHeadingConstraint(Math.toRadians(3))
                                                                .setZeroPowerAccelerationMultiplier(3.5)
                                                                //.addParametricCallback(0.8, ()-> robot.follower.setMaxPower(0.7))
                                                                .build()
                                                ), null))

                        ),



//
//                        // old spec 3 deposit (no samp pickup)
//                        new ParallelCommandGroup(
//                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
//                                // spec 4
//                                new SequentialCommandGroup(
//                                        new WaitCommand(300),
//                                        new FollowPathChainCommand(robot.follower, paths.get(3))
//                                                .alongWith(
//                                                        new WaitCommand(200).andThen(new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)),
//                                                        new WaitCommand(50).andThen(new DepositSpecimenCommand(robot))
//                                                )
//                                )
//                        ),


                        new InstantCommand(()-> robot.follower.setMaxPower(1)),
                        new IntakeSpecimenAutoCommand(robot).alongWith(
                                // Deposit spec 4
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(4))
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                                        )
                                                )
                                )
                        ),

                        new ParallelCommandGroup(
                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                                // spec 5
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(5))
                                                .alongWith(
                                                        new WaitCommand(200).andThen(new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)),
                                                        new WaitCommand(50).andThen(new DepositSpecimenCommand(robot))
                                                )
                                )
                        ),

                        new InstantCommand(()-> robot.follower.setMaxPower(1)),

                        new IntakeSpecimenAutoCommand(robot).alongWith(
                                // Deposit spec 5
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(6))
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                                        )
                                                )
                                )
                        ),
                        new ParallelCommandGroup(
                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                                // spec 5
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(7))
                                                .alongWith(
                                                        new WaitCommand(200).andThen(new LiftCommand(robot, LiftSubsystem.LiftState.INTAKE_SPECIMEN)),
                                                        new WaitCommand(50).andThen(new DepositSpecimenCommand(robot))
                                                )
                                )
                        ),
                        new InstantCommand(()-> robot.follower.setMaxPower(1)),

                        new IntakeSpecimenAutoCommand(robot).alongWith(
                                // Deposit spec 6
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathChainCommand(robot.follower, paths.get(8))
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP)
                                                        )
                                                )
                                )
                        ),
                        new LiftCommand(robot, LiftSubsystem.LiftState.DEPOSIT_HIGH_SPECIMEN),
                        new DepositSpecimenCommand(robot)



                )
        );
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
        robot.telemetryA.addData("lift pos", robot.liftActuator.getPosition());
        robot.telemetryA.addData("extendo pos ticks", robot.intake.getExtendoPosTicks());

        robot.telemetryA.addData("Lift target", robot.liftActuator.getTargetPosition());
        robot.telemetryA.addData("Lift motor powers", robot.liftActuator.getPower());
        robot.telemetryA.addData("t value (general loop)", robot.follower.getCurrentTValue());
        robot.telemetryA.addData("~~PIDF~~", FollowerConstants.translationalPIDFCoefficients.toString());

        FtcDashboard.getInstance().sendImage(robot.vision.draw(320,240));
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

        robot.telemetryA.update();

        loopTime = loop;
        Globals.END_OF_AUTO_POSE = robot.follower.poseUpdater.getPose();
    }
}