package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class DepositSampleCommand extends SequentialCommandGroup {

    public DepositSampleCommand(Robot robot) {
        super(
                new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
//                new WaitCommand(200),
//                new InstantCommand(()-> robot.depositPivotServo.setPosition(0.5025)),
                new InstantCommand(()-> Globals.HOLDING_SAMPLE = false)

        );
    }
}