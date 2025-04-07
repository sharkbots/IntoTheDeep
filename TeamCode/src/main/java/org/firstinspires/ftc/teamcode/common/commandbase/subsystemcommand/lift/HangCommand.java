package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class HangCommand extends SequentialCommandGroup {
    public HangCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.lift.updateState(LiftSubsystem.LiftState.LVL2_ASCENT_DOWN)),
                new WaitUntilCommand(()-> robot.liftActuator.getPosition() < ENDGAME_ASCENT_HEIGHT+200)
        );
    }
}