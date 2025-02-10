package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class AutoIntakeSampleCommand extends SequentialCommandGroup {
    public AutoIntakeSampleCommand(Robot robot) {
        super(
                new SetIntake(robot, IntakeSubsystem.PivotState.INTAKE),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.INTAKE)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(200)
        );
    }
}
