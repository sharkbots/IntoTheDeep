package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand(Robot robot) {
        super(
                new ManualSampleIntakeCommand(robot),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_WITH_SAMPLE))
        );
    }
}