package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
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
                            double targetExtendoPos = robot.intake.getExtendoPosInches() + robot.sampleDetectionPipeline.getCameraYOffset();
                            robot.intake.setExtendoTargetInches(targetExtendoPos);
                        }),

                        // Step 3: Dynamically calculate target claw rotation and set intake
                        // Not dynamically somehow messes up and only calculates it once

                        // TODO: Change InstantCommand() block to setIntake() command below. Use .beforeStarting()
                        new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE, () -> robot.intake.getClawRotationDegrees()+robot.sampleDetectionPipeline.getCameraHeadingOffsetDegrees()),

                        // Step 4: Wait for the extendo to reach its target position
                        new WaitUntilCommand(() -> robot.intake.extendoReached())
                        // TODO: Once dynamic HoldPoint() works, add it below. .alongWith() is to make it a parallel command group.
                ).alongWith(
                        new SequentialCommandGroup(
                                // Step 2: Adjust drivetrain in the x direction based on camera X offset
                                new HoldPointCommand(robot.follower, () -> MathFunctions.addPoses(
                                        new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading()),
                                        MathFunctions.rotatePose(new Pose(Math.copySign(0.7, robot.sampleDetectionPipeline.getCameraXOffset()) + robot.sampleDetectionPipeline.getCameraXOffset(),
                                                0, 0), robot.follower.getPose().getHeading()-Math.PI/2, false))
                                )/*.alongWith(
                                        new WaitCommand(450)
                                )*/,
                                new InstantCommand(() -> robot.follower.startTeleopDrive())
                        )

                ),

                // Step 5: Close the claw
//                new WaitCommand(30),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),

                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
                new WaitCommand(230),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE))
        );
    }
}