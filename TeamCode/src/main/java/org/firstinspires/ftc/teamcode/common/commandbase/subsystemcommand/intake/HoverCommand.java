package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class HoverCommand extends SequentialCommandGroup {
    public HoverCommand(Robot robot, double extension) {
        super(
                new InstantCommand(() -> robot.extendoActuator.setTargetPosition(extension)),
                new SetIntake(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE),
                new WaitUntilCommand(()-> robot.intake.extendoReached()),
                //new InstantCommand(()-> robot.intakeClawLED.setPwmEnable()),

                /*new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE)),
                new WaitCommand(600),*/
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = true)
        );
    }
}