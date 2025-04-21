package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class IntakeSpecimenAutoCommand extends SequentialCommandGroup {
    public IntakeSpecimenAutoCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.CLOSED)),
                new WaitCommand(200),
                new InstantCommand(()-> robot.lift.updateState(LiftSubsystem.LiftState.DEPOSIT_HIGH_RUNG_SETUP, 0.0)),
                new InstantCommand(()-> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = true)
        );
    }
}
