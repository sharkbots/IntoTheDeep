package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class ReGrabSampleCommand extends SequentialCommandGroup {
    public ReGrabSampleCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new SetIntake(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE),
                //new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE)),
                new WaitCommand(20)
        );
    }
}
