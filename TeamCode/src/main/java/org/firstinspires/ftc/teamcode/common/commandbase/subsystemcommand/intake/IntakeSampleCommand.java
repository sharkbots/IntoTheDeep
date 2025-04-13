package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand(Robot robot) {
        super(
                new InstantCommand(()->robot.intake.setPivotState(IntakeSubsystem.PivotState.INTAKE)),
                new DeferredCommand(()->
                        new InstantCommand(()->robot.intake.setClawRotationDegrees(robot.intake.getClawRotationDegrees()))
                        , null
                ),
                new WaitCommand(80),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(230)
        );
    }
}