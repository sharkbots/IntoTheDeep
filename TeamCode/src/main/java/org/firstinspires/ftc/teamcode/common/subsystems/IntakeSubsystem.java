package org.firstinspires.ftc.teamcode.common.subsystems;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public PivotState pivotState = PivotState.TRANSFER;


    public ClawState clawState = ClawState.OPEN;

    public enum PivotState{
        HOVERING,
        INTAKING,
        TRANSFER
    }

    public enum ClawState{
        OPEN(0.0),
        CLOSED(1.0);

        ClawState(double v) {
        }
    }

    public IntakeSubsystem (){
        this.robot = Robot.getInstance();

    }

    public ClawState getClawState() {
       return clawState;
    }

    public double getClawStatePos(ClawState state) {return state.ordinal();}

    public void updateState(@NotNull ClawState state) {
        this.clawState = state;
        robot.intakeClawServo.setPosition(getClawStatePos(state));
    }

    public void updateState(@NotNull PivotState state) {this.pivotState = state; }

    @Override
    public void periodic() {
        robot.extendoActuator.periodic();
        robot.intakeArmPivotActuator.periodic();
        robot.intakeClawPivotActuator.periodic();
    }

    @Override
    public void read() {
        robot.extendoActuator.read();
        robot.intakeArmPivotActuator.read();
        robot.intakeClawPivotActuator.read();
    }

    @Override
    public void write() {
        robot.extendoActuator.write();
        robot.intakeArmPivotActuator.write();
        robot.intakeClawPivotActuator.write();
    }

    @Override
    public void reset() {

    }
}
