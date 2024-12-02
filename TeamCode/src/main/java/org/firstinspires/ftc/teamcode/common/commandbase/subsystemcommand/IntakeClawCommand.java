package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(Robot robot, IntakeSubsystem.ClawState state) {
        super(
                () -> robot.intake.setClawState(state)
        );
    }
}