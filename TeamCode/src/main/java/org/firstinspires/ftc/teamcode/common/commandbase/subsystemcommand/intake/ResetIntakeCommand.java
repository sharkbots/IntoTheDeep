package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ResetIntakeCommand extends SequentialCommandGroup {
    public ResetIntakeCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.intake.setExtendoTargetTicks(0)),
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.TRANSFER),
                //new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.TRANSFER)),
                //new ClawRotationCommand(robot, IntakeSubsystem.ClawRotationState.TRANSFER),
                new WaitUntilCommand(() -> robot.intake.extendoReached()),
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = false)
        );
    }
}
