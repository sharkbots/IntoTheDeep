package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ResetLiftCommand extends SequentialCommandGroup {
    public ResetLiftCommand(Robot robot){
        super(
                new InstantCommand(() -> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = false),
                new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
        );
    }
}
