package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;

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
                        new InstantCommand(() -> {
                            // Calculate target claw rotation dynamically
                            double currentClawRotation = robot.intake.getClawRotationDegrees();
                            double cameraOffset = robot.sampleDetectionPipeline.getCameraHeadingOffsetDegrees();
                            double targetClawRotation = currentClawRotation + cameraOffset;

                            // Set the intake pivot state and claw rotation
                            new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE, targetClawRotation).schedule();
                        }),

                        // Step 4: Wait for the extendo to reach its target position
                        new WaitUntilCommand(() -> robot.intake.extendoReached())
                )/*.alongWith(
                        // Step 2: Adjust drivetrain in the x direction based on camera X offset
                        new InstantCommand(() -> {
                            Pose targetDtPose = new Pose(robot.follower.getPose().getX()+robot.sampleDetectionPipeline.getCameraXOffset(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading());
                            new HoldPointCommand(robot.follower, targetDtPose).schedule();
                        })
                )*/,

                // Step 5: Close the claw
                new WaitCommand(30),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),

                // Step 6: Wait and transition to HOVERING_WITH_SAMPLE
                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE))
        );
    }
}