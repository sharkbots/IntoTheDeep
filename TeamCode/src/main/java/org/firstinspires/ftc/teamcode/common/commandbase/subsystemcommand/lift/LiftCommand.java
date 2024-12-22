package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private final Robot robot;
    private final LiftSubsystem.LiftState liftState;

    public LiftCommand(Robot robot, LiftSubsystem.LiftState state) {
        this.robot = robot;
        this.liftState = state;
    }

    @Override
    public void initialize(){
        robot.lift.updateState(liftState);
    }

    @Override
    public boolean isFinished(){
        return robot.lift.isActuatorAtTarget();
    }
}