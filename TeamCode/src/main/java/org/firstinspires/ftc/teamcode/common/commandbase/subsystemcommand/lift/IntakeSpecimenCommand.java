package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import android.provider.Settings;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class IntakeSpecimenCommand extends SequentialCommandGroup {
    public IntakeSpecimenCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.CLOSED)),
                new WaitCommand(200),
                new LiftCommand(robot, LiftSubsystem.LiftState.HOLDING_SPECIMEN),
                new InstantCommand(()-> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = true)
                );
    }

}