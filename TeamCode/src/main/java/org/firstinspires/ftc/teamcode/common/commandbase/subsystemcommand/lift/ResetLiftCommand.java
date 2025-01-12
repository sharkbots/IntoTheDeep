package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ResetLiftCommand extends SequentialCommandGroup {
    public ResetLiftCommand(Robot robot){
        super(
                new InstantCommand(robot.lift::reset),
                new InstantCommand(() -> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = false)

        );
    }
}
