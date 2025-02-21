package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class AutomaticIntakeCommand extends SequentialCommandGroup {
    public AutomaticIntakeCommand(Robot robot) {
        super(
                new HoverCommand(robot, 0),
                new WaitUntilCommand(()->robot.sampleDetectionPipeline.cameraXInRange()),
                new IntakeCommand(robot),
                new TransferCommand(robot)
        );
    }
}
