package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.MICRO_OPEN)),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.intake.setExtendoTarget(100)),
                new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.TRANSFER)),
                new ClawRotationCommand(robot, IntakeSubsystem.ClawRotationState.TRANSFER),
                new WaitCommand(300),
                new WaitUntilCommand(() -> robot.intake.extendoReached()),
                new InstantCommand(() -> robot.intake.setExtendoTarget(0)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.CLOSED)),
                new WaitCommand(200),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = true)
        );
    }
}