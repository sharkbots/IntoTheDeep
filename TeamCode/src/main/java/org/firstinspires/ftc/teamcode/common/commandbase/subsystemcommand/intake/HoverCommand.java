package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class HoverCommand extends SequentialCommandGroup {
    public HoverCommand(Robot robot, double extension, Double clawRotationDegrees) {
        super(
                new InstantCommand(() -> robot.intake.setExtendoTargetTicks((int)extension)),
                new ConditionalCommand(
                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE, clawRotationDegrees),
                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL, clawRotationDegrees),
                        ()-> Globals.GRABBING_MODE != Globals.GRABBING_MODES.MANUAL
                ),
                new WaitUntilCommand(()-> robot.intake.extendoReached()),
                //new InstantCommand(()-> robot.intakeClawLED.setPwmEnable()),

                /*new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE)),
                new WaitCommand(600),*/
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = true)
        );
    }

    public HoverCommand(Robot robot, double extension) {
        this(robot, extension, (Double) null);
    }
}