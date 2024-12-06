package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot  .intake.setPivotState(IntakeSubsystem.PivotState.INTAKING)),
                new WaitCommand(100),
                /*new WaitUntilCommand(robot.intake::pivotReached),*/
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.MICRO_OPEN)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.intake.setExtendoTarget(200)),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.TRANSFER)),
                new ClawRotationCommand(robot, IntakeSubsystem.ClawRotationState.TRANSFER),
                new WaitCommand(1000),
                new WaitUntilCommand(() -> /*robot.intake.pivotReached() &&*/robot.intake.extendoReached()),
                new InstantCommand(() -> robot.intake.setExtendoTarget(0)),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.MICRO_OPEN)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.CLOSED)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> Globals.INTAKING = false)
        );
    }
}