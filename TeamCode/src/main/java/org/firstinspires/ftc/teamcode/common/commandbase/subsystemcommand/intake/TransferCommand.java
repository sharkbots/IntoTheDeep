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
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                //new ClawRotationCommand(robot, IntakeSubsystem.ClawRotationState.TRANSFER),
                new InstantCommand(() -> robot.intake.setExtendoTargetTicks(0)),
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.TRANSFER).alongWith(
                        new SequentialCommandGroup(
                                new WaitCommand(460),
                                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.MICRO_OPEN))
                        )
                ),
                new WaitUntilCommand(() -> robot.intake.extendoReached()),
//                new InstantCommand(() -> robot.intake.setExtendoTargetTicks(0)),
//                new WaitUntilCommand(() -> robot.intake.extendoReached()), /*prev wait 350*/
                new WaitCommand(35),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.CLOSED)),
                new WaitCommand(90),
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = true)
        );
    }
}