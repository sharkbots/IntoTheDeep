package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ReGrabSampleCommand extends SequentialCommandGroup {
    public ReGrabSampleCommand(Robot robot){
        super(
                new InstantCommand(() -> robot.intake.setClawState(IntakeSubsystem.ClawState.OPEN)),
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL, 0.0),
//                new ConditionalCommand(
//                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE),
//                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL),
//                        ()-> Globals.GRABBING_MODES.current() != Globals.GRABBING_MODES.MANUAL
//                ),
                //new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE)),
                new WaitCommand(20)
        );
    }
}
