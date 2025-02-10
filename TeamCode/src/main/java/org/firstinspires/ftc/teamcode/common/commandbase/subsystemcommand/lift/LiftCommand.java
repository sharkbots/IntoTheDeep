package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Robot robot, LiftSubsystem.LiftState state) {
        super(
                new InstantCommand(() -> robot.lift.updateState(state)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(LIFT_RESET_FEEDFORWARD)),
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(DEFAULT_LIFT_FEEDFORWARD)),
                        () -> (state == LiftSubsystem.LiftState.RETRACTED || state == LiftSubsystem.LiftState.INTAKE_SPECIMEN)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(LIFT_NEAR_RESET_FEEDFORWARD)),
                        new InstantCommand(),
                        () -> (state == LiftSubsystem.LiftState.HOLDING_SPECIMEN)),
                new WaitUntilCommand(()->robot.lift.liftReached())
                );
    }
}