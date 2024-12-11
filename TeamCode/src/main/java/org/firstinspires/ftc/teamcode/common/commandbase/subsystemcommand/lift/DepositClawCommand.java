package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

public class DepositClawCommand extends InstantCommand {

    public DepositClawCommand(Robot robot, LiftSubsystem.ClawState state) {
        super(() -> robot.lift.updateState(state)
        );
    }
}