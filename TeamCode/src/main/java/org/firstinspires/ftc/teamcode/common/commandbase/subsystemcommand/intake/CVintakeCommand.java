package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class CVintakeCommand extends SequentialCommandGroup {
    public CVintakeCommand(Robot robot) {
        super(
                new HoverCommand(robot, 0),
                new WaitUntilCommand(()->robot.sampleDetectionPipeline.cameraInRange()),
                new AutoIntakeSampleCommand(robot),
                new TransferCommand(robot)
        );
    }
}
