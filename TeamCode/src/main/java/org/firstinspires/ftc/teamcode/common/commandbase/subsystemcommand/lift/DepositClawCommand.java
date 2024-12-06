package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

public class DepositClawCommand extends ConditionalCommand {

    public DepositClawCommand(Robot robot, LiftSubsystem.ClawState state) {
        super(
                // Run if claw control is allowed
                new InstantCommand(() -> robot.lift.updateState(state)),

                // Do nothing otherwise
                new InstantCommand(),

                robot.lift::isClawControlAllowed
        );
    }
}