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
        MICRO_OPEN(0.0),
        CLOSED(1.0);

        private final double position;
        ClawState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public IntakeSubsystem (){
        this.robot = Robot.getInstance();


    }

    /**
     * Sets the pivot state for the arm and claw.
     */
    public void setPivotState(@NotNull PivotState state) {
        this.pivotState = state;

        robot.intakeArmPivotActuator.setTargetPosition(getArmPivotPosition(state));
        robot.intakeClawPivotActuator.setTargetPosition(getClawPivotPosition(state));
    }

    /**
     * Sets the claw state (OPEN or CLOSED).
     */
    public void setClawState(@NotNull ClawState state) {
        this.clawState = state;
        robot.intakeClawServo.setPosition(state.getPosition());
    }

    /**
     * Sets the claw rotation state.
     */
    public void setClawRotationState(double position) {
        robot.intakeClawRotationServo.setPosition(position);
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public ClawState getClawState() {
        return clawState;
    }

    /**
     * Retrieves the claw pivot servo position for the given pivot state.
     */
    private double getClawPivotPosition(PivotState state) {
        switch (state) {
            case TRANSFER:
                return 0.24;
            case HOVERING:
                return 0.95;
            case INTAKING:
                return 0.91;
            default: throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

    /**
     * Retrieves the arm pivot servo position for the given pivot state.
     */
    private double getArmPivotPosition(PivotState state) {
        switch (state) {
            case TRANSFER:
                return 0.47;
            case HOVERING:
                return 0.74;
            case INTAKING:
                return 0.78;
            default: throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

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
