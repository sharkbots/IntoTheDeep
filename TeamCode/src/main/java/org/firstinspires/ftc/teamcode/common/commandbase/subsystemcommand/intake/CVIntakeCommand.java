package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;

import org.firstinspires.ftc.teamcode.common.commandbase.HoldPointCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class CVIntakeCommand extends SequentialCommandGroup {
    public CVIntakeCommand(Robot robot) {
        // Build the command sequence
        super(
                //new InstantCommand(()->robot.visionPortal.setProcessorEnabled(robot.sampleDetectionPipeline, false)),
                //new InstantCommand(()->robot.sampleDetectionPipeline.freeze(true)),
                new InstantCommand(()->robot.sampleDetectionPipeline.setLatestValidCenter()),
                new SequentialCommandGroup(
                        // Step 1: Adjust the extendo position based on camera Y offset
                        new InstantCommand(() -> {
                            double targetExtendoPos = robot.intake.getExtendoPosInches() + robot.sampleDetectionPipeline.getLatestValidCameraYOffset() - Globals.CAMERA_OFFSET_FROM_CENTER_Y_IN;

                            robot.intake.setExtendoTargetInches(targetExtendoPos);

                        }),
                        new InstantCommand(()-> {
                            robot.telemetryA.addData("extendo target pos (intake)", robot.extendoActuator.getTargetPosition());
                        }),

                        // Step 4: Wait for the extendo to reach its target position
                        new ParallelRaceGroup(
                                new WaitUntilCommand(() -> robot.intake.extendoReached()),
                                new WaitCommand(1200)
                        )
                        // TODO: Once dynamic HoldPoint() works, add it below. .alongWith() is to make it a parallel command group.
                ).alongWith(
                        new SequentialCommandGroup(
                                // Step 2: Adjust drivetrain in the x direction based on camera X offset
                                new HoldPointCommand(robot.follower, () -> MathFunctions.addPoses(
                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading()),
                                        MathFunctions.rotatePose(new Pose(robot.sampleDetectionPipeline.getLatestValidCameraXOffset(),
                                                0, 0), robot.follower.getPose().getHeading()-Math.PI/2, false))
                                ),
                                new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE, () -> robot.intake.getClawRotationDegrees()+robot.sampleDetectionPipeline.getLatestValidCameraHeadingOffsetDegrees()),
                                new InstantCommand(() -> robot.follower.startTeleopDrive())
                        )

                ),

                // Step 5: Close the claw
//                new WaitCommand(30),
                new SequentialCommandGroup(
                new InstantCommand(()-> {
                    robot.telemetryA.addData("about to close claw (cv intake)", 0);
                    robot.telemetryA.update();}),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),

                new InstantCommand(()->{
                    robot.telemetryA.addData("claw closed (cv intake)", 0);
                    robot.telemetryA.update();
                }),
                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
                new WaitCommand(230),

                new InstantCommand(()-> {
                    robot.telemetryA.addData("about to pivot up (cv intake)", 0);
                    robot.telemetryA.update();}),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)),
                new InstantCommand(()->{
                    robot.telemetryA.addData("sent pivot command (cv intake)", 0);
                    robot.telemetryA.update();
                })
        )
            //    new InstantCommand(()->robot.visionPortal.setProcessorEnabled(robot.sampleDetectionPipeline, true))
//        new InstantCommand(()->robot.sampleDetectionPipeline.freeze(false))

        );
    }
}