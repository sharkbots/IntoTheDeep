package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.DEFAULT_LIFT_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.LIFT_NEAR_RESET_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.LIFT_RESET_FEEDFORWARD;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ResetLiftCommand extends SequentialCommandGroup {
    public ResetLiftCommand(Robot robot){
        super(
                new InstantCommand(() -> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = false),
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.ClawState.OPEN)),
                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
        );
    }
}
