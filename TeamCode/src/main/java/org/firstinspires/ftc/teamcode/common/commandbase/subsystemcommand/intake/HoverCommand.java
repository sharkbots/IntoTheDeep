package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class HoverCommand extends SequentialCommandGroup {
    public HoverCommand(Robot robot, double extension, Double clawRotationDegrees) {
        super(
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = true),
                new InstantCommand(() -> robot.intake.setExtendoTargetTicks((int)extension)),
                new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL, clawRotationDegrees),
//                new ConditionalCommand(
//                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE, clawRotationDegrees),
//                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL, clawRotationDegrees),
//                        ()-> Globals.GRABBING_MODES.current() != Globals.GRABBING_MODES.MANUAL
//                ),
                new WaitUntilCommand(()-> robot.intake.extendoReached())
                //new InstantCommand(()-> robot.intakeClawLED.setPwmEnable()),

                /*new InstantCommand(() -> robot.intake.setPivotState(IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE)),
                new WaitCommand(600),*/
        );
    }

    public HoverCommand(Robot robot, double extension) {
        this(robot, extension, (Double) null);
    }
}