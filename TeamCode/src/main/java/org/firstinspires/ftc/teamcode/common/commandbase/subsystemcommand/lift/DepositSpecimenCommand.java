package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class DepositSpecimenCommand extends SequentialCommandGroup {
    public DepositSpecimenCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = false)
        );
    }
}