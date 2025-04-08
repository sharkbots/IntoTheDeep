package org.firstinspires.ftc.teamcode.common.utils.commandBase;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

public class FixedSequentialCommandGroup extends SequentialCommandGroup {
    private boolean finished;

    public FixedSequentialCommandGroup(Command... commands) {
        super(commands);
    }

    @Override
    public void initialize() {
        super.initialize();
        finished = false;
    }

    @Override
    public void execute() {
        if (finished)
            return;

        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (finished)
            return;

        super.end(interrupted);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        finished |= super.isFinished();
        return finished;
    }
}