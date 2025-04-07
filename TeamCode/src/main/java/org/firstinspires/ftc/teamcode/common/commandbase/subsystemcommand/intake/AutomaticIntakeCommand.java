package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AutomaticIntakeCommand extends SequentialCommandGroup {
    public AutomaticIntakeCommand(Robot robot) {
        super(
                new HoverCommand(robot, 0),
                new WaitUntilCommand(()->robot.sampleDetectionPipeline.cameraXInRange()),
                new ManualSampleIntakeCommand(robot),
                new TransferCommand(robot)
        );
    }
}
