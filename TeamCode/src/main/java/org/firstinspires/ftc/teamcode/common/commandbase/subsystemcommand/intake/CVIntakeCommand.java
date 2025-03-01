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
        addCommands(
                new SequentialCommandGroup(
                        // Step 1: Adjust the extendo position based on camera Y offset
                        new InstantCommand(() -> {
                            double targetExtendoPos = robot.intake.getExtendoPosInches() + robot.sampleDetectionPipeline.getCameraYOffset() - Globals.CAMERA_OFFSET_FROM_CENTER_Y_IN;
//                            if (robot.intake.getExtendoPosTicks() > 660){
//                                targetExtendoPos += (robot.intake.getExtendoPosTicks()-660)*(-0.0805)/Globals.EXTENDO_TICKS_PER_INCH;
//                            }
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
                                        MathFunctions.rotatePose(new Pose(robot.sampleDetectionPipeline.getCameraXOffset(),
                                                0, 0), robot.follower.getPose().getHeading()-Math.PI/2, false))
                                ),
                                new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE, () -> robot.intake.getClawRotationDegrees()+robot.sampleDetectionPipeline.getCameraHeadingOffsetDegrees()),
                                new InstantCommand(() -> robot.follower.startTeleopDrive())
                        )

                ),

                // Step 5: Close the claw
//                new WaitCommand(30),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),

                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
                new WaitCommand(230),
                // REMOVE ASAP
                //new InstantCommand(()-> robot.intake.pivotState = IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE),
                //new SetIntake(robot, IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE))
        );
    }
}