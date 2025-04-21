package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake.SetIntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class ResetLiftCommand extends SequentialCommandGroup {
    private final Robot r;
    public ResetLiftCommand(Robot robot){
        super(
                new InstantCommand(() -> Globals.INTAKING_SPECIMENS = false),
                new InstantCommand(() -> Globals.HOLDING_SAMPLE = false),
                new InstantCommand(() -> Globals.HOLDING_SPECIMEN = false),
                new InstantCommand(() -> robot.lift.setClawState(LiftSubsystem.ClawState.OPEN)),
                new LiftCommand(robot, LiftSubsystem.LiftState.RETRACTED)
        );
        this.r = robot;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            new DeferredCommand(()->new SetIntakeCommand(r, IntakeSubsystem.PivotState.TRANSFER,
                    0.0), null).alongWith(
                            new ParallelCommandGroup(
                                    new LiftCommand(r, LiftSubsystem.LiftState.INTAKE_SPECIMEN),
                                    new InstantCommand(()->{
                                            Globals.INTAKING_SPECIMENS = true;
                                            Globals.INTAKING_SAMPLES = false;
                                            r.intake.setExtendoTargetTicks(0);
                                    })
                            )
            ).schedule();
        }
    }
}
