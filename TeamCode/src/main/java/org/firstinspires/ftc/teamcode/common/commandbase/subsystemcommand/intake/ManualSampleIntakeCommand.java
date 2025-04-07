package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class ManualSampleIntakeCommand extends SequentialCommandGroup {
    public ManualSampleIntakeCommand(Robot robot) {
        super(
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.INTAKE, ()-> robot.intake.getClawRotationDegrees()),
                //new WaitCommand(30),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(230),
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE)
        );
    }
}
