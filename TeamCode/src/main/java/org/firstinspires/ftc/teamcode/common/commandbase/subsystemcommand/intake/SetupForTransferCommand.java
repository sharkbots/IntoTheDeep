package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift.LiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class SetupForTransferCommand extends SequentialCommandGroup {
    private final Robot r;

    public SetupForTransferCommand(Robot robot, boolean delayRotation){
        super(
                new InstantCommand(() -> Globals.INTAKING_SAMPLES = false),
                new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN_TRANSFER)),
                //new ClawRotationCommand(robot, IntakeSubsystem.ClawRotationState.TRANSFER),
                new InstantCommand(() -> robot.intake.setExtendoTargetTicks(0)),
                new ConditionalCommand(
                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.TRANSFER),
                        new SetIntakeCommand(robot, IntakeSubsystem.PivotState.TRANSFER, 0.0),
                        ()-> delayRotation
                ),
                new WaitUntilCommand(() -> robot.intake.extendoReached() || robot.intake.getExtendoPosTicks() < 50),
                new WaitCommand(200),
                new ConditionalCommand(
                        new InstantCommand(()-> robot.intake.setClawRotation(IntakeSubsystem.PivotState.TRANSFER)),
                        new InstantCommand(),
                        ()-> delayRotation
                )
        );
        this.r = robot;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) { // Recall Last intake position
            new DeferredCommand(()->new SetIntakeCommand(r, IntakeSubsystem.PivotState.HOVERING_NO_SAMPLE_MANUAL,
                    r.intake.getPreviousClawRotation()), null).alongWith(
                    new InstantCommand(()->{
                        //INTAKE_JUST_CANCELLED = false;
                        Globals.INTAKING_SAMPLES = true;
                        r.intake.setClawState(IntakeSubsystem.ClawState.OPEN);
                        r.intake.setExtendoTargetTicks((int)r.intake.getPreviousExtendoTarget());
                    }),
                    new LiftCommand(r, LiftSubsystem.LiftState.RETRACTED)
            ).schedule();
        }
    }
    public SetupForTransferCommand(Robot robot){
        this(robot, false);
    }
}