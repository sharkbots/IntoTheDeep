package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class IntakeSpecimenCommand extends SequentialCommandGroup {
    public IntakeSpecimenCommand(Robot robot){
        super(
                new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.MICRO_OPEN)),
                new WaitCommand(200),
                new LiftCommand(robot, LiftSubsystem.LiftState.HOLDING_SPECIMEN),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = true)
                );
    }

}