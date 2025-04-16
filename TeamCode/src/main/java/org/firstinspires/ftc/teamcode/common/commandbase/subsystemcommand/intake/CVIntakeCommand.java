package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import android.provider.Settings;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;

import org.firstinspires.ftc.teamcode.common.commandbase.HoldPointCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.vision.Sample;

public class CVIntakeCommand extends SequentialCommandGroup {
    public CVIntakeCommand(Robot robot, Sample.Color color) {
        // Build the command sequence
        super(
                //new InstantCommand(()->robot.visionPortal.setProcessorEnabled(robot.sampleDetectionPipeline, false)),
                //new InstantCommand(()->robot.sampleDetectionPipeline.freeze(true)),
                new RunCommand(
                        ()-> robot.vision.detect(color))
                        // .withTimeout(2000) // TODO: activate when all is  so that it is fully protected against infinite loop
                        .interruptOn(() -> !robot.vision.samples().isEmpty() && robot.vision.selectedSample().color() == color ).withTimeout(500),
                //new InstantCommand(()-> Globals.FREEZE_CAMERA_FRAME = true),
                new ParallelCommandGroup(
                        new DeferredCommand(()-> new HoverCommand(robot,
                                (robot.vision.selected()[0] - Globals.INTAKE_MINIMUM_EXTENSION) * Globals.EXTENDO_TICKS_PER_INCH), null),
                        new DeferredCommand(()-> new SetIntakeCommand(robot,
                                IntakeSubsystem.PivotState.INTAKE, (double)(robot.vision.selected()[2])), null),

                        new DeferredCommand(()-> new SequentialCommandGroup(
                                new HoldPointCommand(robot.follower, () -> MathFunctions.addPoses(
                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading()),
                                        MathFunctions.rotatePose(new Pose(-robot.vision.selected()[1],
                                                0, 0), robot.follower.getPose().getHeading()-Math.PI/2, false))
                                ),
                                new InstantCommand(()->robot.follower.startTeleopDrive())
                        ),null)
                ),
                new IntakeSampleCommand(robot),
                new InstantCommand(()-> robot.vision.clear())
//
//                //new WaitCommand(3000),
//                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.INTAKE),
//                new WaitCommand(30),
////                new InstantCommand(()-> {
////                    robot.telemetryA.addData("about to close claw (cv intake)", 0);
////                    robot.telemetryA.update();}),
//                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
////
////                new InstantCommand(()->{
////                    robot.telemetryA.addData("claw closed (cv intake)", 0);
////                    robot.telemetryA.update();
////                }),
//                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
//                new WaitCommand(230),
////
////                new InstantCommand(()-> {
////                    robot.telemetryA.addData("about to pivot up (cv intake)", 0);
////                    robot.telemetryA.update();}),
//                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL)



               // new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)),
//                new InstantCommand(()->{
//                    robot.telemetryA.addData("sent pivot command (cv intake)", 0);
//                    robot.telemetryA.update();
//                })
//                                new HoldPointCommand(robot.follower, new Pose(robot.follower.getPose().getX()-robot.vision.limelight.getClosestOffset()[1], robot.follower.getPose().getY())),
//                                new InstantCommand(() -> robot.follower.startTeleopDrive())), null)
                                // ,
                //new InstantCommand(()-> Globals.FREEZE_CAMERA_FRAME = false)
                //new InstantCommand(()->robot.vision.limelight.setLatestResults() .sampleDetectionPipeline.setLatestValidCenter()),
//                new SequentialCommandGroup(
//                        // Step 1: Adjust the extendo position based on camera Y offset
//                        new InstantCommand(() -> {
//                            double targetExtendoPos = robot.intake.getExtendoPosInches() +  robot.sampleDetectionPipeline.getLatestValidCameraYOffset() - Globals.CAMERA_OFFSET_FROM_CENTER_Y_IN;
//
//                            robot.intake.setExtendoTargetInches(targetExtendoPos);
//
//                        }),
//                        new InstantCommand(()-> {
//                            robot.telemetryA.addData("extendo target pos (intake)", robot.extendoActuator.getTargetPosition());
//                        }),
//
//                        // Step 4: Wait for the extendo to reach its target position
//                        new ParallelRaceGroup(
//                                new WaitUntilCommand(() -> robot.intake.extendoReached()),
//                                new WaitCommand(1200)
//                        )
//                        // TODO: Once dynamic HoldPoint() works, add it below. .alongWith() is to make it a parallel command group.
//                )
//                .alongWith(
//                        new SequentialCommandGroup(
//                                // Step 2: Adjust drivetrain in the x direction based on camera X offset
//                                new HoldPointCommand(robot.follower, () -> MathFunctions.addPoses(
//                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading()),
//                                        MathFunctions.rotatePose(new Pose(robot.sampleDetectionPipeline.getLatestValidCameraXOffset(),
//                                                0, 0), robot.follower.getPose().getHeading()-Math.PI/2, false))
//                                ),
//                                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.INTAKE, () -> robot.intake.getClawRotationDegrees()+robot.sampleDetectionPipeline.getLatestValidCameraHeadingOffsetDegrees()),
//                                new InstantCommand(() -> robot.follower.startTeleopDrive())
//                        )
//
//                ),

                // Step 5: Close the claw
//                new WaitCommand(30),
//                new SequentialCommandGroup(
//                new InstantCommand(()-> {
//                    robot.telemetryA.addData("about to close claw (cv intake)", 0);
//                    robot.telemetryA.update();}),
//                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
//
//                new InstantCommand(()->{
//                    robot.telemetryA.addData("claw closed (cv intake)", 0);
//                    robot.telemetryA.update();
//                }),
//                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
//                new WaitCommand(230),
//
//                new InstantCommand(()-> {
//                    robot.telemetryA.addData("about to pivot up (cv intake)", 0);
//                    robot.telemetryA.update();}),
//                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE),
//               // new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)),
//                new InstantCommand(()->{
//                    robot.telemetryA.addData("sent pivot command (cv intake)", 0);
//                    robot.telemetryA.update();
//                })
            //    new InstantCommand(()->robot.visionPortal.setProcessorEnabled(robot.sampleDetectionPipeline, true))
//        new InstantCommand(()->robot.sampleDetectionPipeline.freeze(false))
        );
    }
}