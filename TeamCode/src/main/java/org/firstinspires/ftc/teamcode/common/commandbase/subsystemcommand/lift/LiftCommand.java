package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.lift;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Robot robot, LiftSubsystem.LiftState state) {
        super(
                new ConditionalCommand(
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(LIFT_RESET_FEEDFORWARD)),
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(DEFAULT_LIFT_FEEDFORWARD)),
                        () -> (state == LiftSubsystem.LiftState.RETRACTED || state == LiftSubsystem.LiftState.INTAKE_SPECIMEN || state == LiftSubsystem.LiftState.TRANSFER)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.liftActuator.updateFeedforward(LIFT_NEAR_RESET_FEEDFORWARD)),
                        new InstantCommand(),
                        () -> (state == LiftSubsystem.LiftState.HOLDING_SPECIMEN)),

                new InstantCommand(() -> robot.lift.updateState(state))
                        .alongWith(
                                new ConditionalCommand(
                                        new WaitCommand(200).andThen(new InstantCommand(()-> robot.lift.setClawState(LiftSubsystem.ClawState.MICRO_OPEN))),
                                        new InstantCommand(),
                                        ()-> state == LiftSubsystem.LiftState.DEPOSIT_LOW_BUCKET || state == LiftSubsystem.LiftState.DEPOSIT_HIGH_BUCKET
                                )
                        ),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                            new WaitUntilCommand(()-> robot.liftActuator.getPosition() < 200),
                            new WaitCommand(400),
                            new InstantCommand(()-> {
                                robot.liftTopEncoder.reset();
                        })),

//                new WaitUntilCommand(()-> robot.liftActuator.getPosition() < 200),
//                new InstantCommand(()-> {
//                    robot.lift.isResetting = true;
//                    robot.liftActuator.enableManualPower();
//                    robot.liftActuator.setOverridePower(-1);
//                }),
//                new WaitUntilCommand(()-> robot.liftBottomMotor.isOverCurrent()),
//                new InstantCommand(()->{
//                    robot.liftActuator.setOverridePower(0);
//                }),
//                new WaitCommand(300),
//                new InstantCommand(()-> {
//                    robot.liftTopEncoder.reset();
//                    robot.lift.isResetting = false;
//                })

                        new WaitUntilCommand(()->robot.lift.liftReached()),
                        () -> state == LiftSubsystem.LiftState.RETRACTED || state == LiftSubsystem.LiftState.INTAKE_SPECIMEN || state == LiftSubsystem.LiftState.TRANSFER
                )

                );
    }
}