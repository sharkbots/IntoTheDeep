package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class CVIntakeCommand extends SequentialCommandGroup {
    public CVIntakeCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.intake.setExtendoTargetInches(robot.sampleDetectionPipeline.getCameraYOffset())),
                new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE, robot.sampleDetectionPipeline.getCameraHeadingOffsetDegrees()),
                new WaitUntilCommand(()-> robot.intake.extendoReached()),
                new WaitCommand(30),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(200)
        );
    }
}
