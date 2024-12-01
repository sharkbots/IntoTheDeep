package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

public class LiftCommand extends SequentialCommandGroup {

    public LiftCommand(Robot robot, LiftSubsystem.LiftState state) {
        super(
                // Set the lift state
                new InstantCommand(() -> robot.lift.updateState(state)),

                // Wait until the actuator reaches the target position
                new WaitUntilCommand(robot.lift::isActuatorAtTarget)
        );
    }
}