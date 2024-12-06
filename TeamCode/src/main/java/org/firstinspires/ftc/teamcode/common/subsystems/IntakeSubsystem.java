package org.firstinspires.ftc.teamcode.common.subsystems;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public PivotState pivotState = PivotState.TRANSFER;
    public ClawState clawState = ClawState.OPEN;

    public int extendoTargetPos = 1500;

    public enum PivotState{
        HOVERING,
        INTAKING,
        TRANSFER
    }

    public enum ClawRotationState{
        TRANSFER(0.54),
        FULLY_LEFT(0.88),
        FULLY_RIGHT(0.195);

        private final double position;
        ClawRotationState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum ClawState{
        OPEN(0.75),
        MICRO_OPEN(0.553),
        CLOSED(0.5);

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
        robot.extendoActuator.setTargetPosition(0);
        setPivotState(PivotState.TRANSFER);
        setClawState(ClawState.OPEN);
        setClawRotation(ClawRotationState.TRANSFER);
    }


    public void setExtendoTarget(int pos){
        this.extendoTargetPos = pos;
        robot.extendoActuator.setTargetPosition(pos);
    }

    /**
     * Sets the pivot state for the arm and claw.
     * @param state Pivot state
     */
    public void setPivotState(@NotNull PivotState state) {
        this.pivotState = state;
        robot.intakeArmPivotLeftServo.setPosition(getArmPivotPosition(state));
        robot.intakeArmPivotRightServo.setPosition(getArmPivotPosition(state));
        robot.intakeClawPivotServo.setPosition(getClawPivotPosition(state));
    }

    /**
     * Sets the claw state (OPEN or CLOSED).
     * @param state Claw State
     */
    public void setClawState(@NotNull ClawState state) {
        this.clawState = state;
        robot.intakeClawServo.setPosition(state.getPosition());
    }


    public void setClawRotation(@NotNull ClawRotationState state) {
        robot.intakeClawRotationServo.setPosition(state.getPosition());
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
                return 0.89;
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
                return 0.79;
            default: throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

    public boolean pivotReached(){
        return robot.intakeArmPivotActuator.hasReached() && robot.intakeClawPivotActuator.hasReached();
    }

    public boolean extendoReached(){
        return robot.extendoActuator.hasReached();
    }

    @Override
    public void periodic() {
        //robot.extendoActuator.setTargetPosition(extendoTargetPos);
//        if (pivotState == PivotState.TRANSFER){
//            robot.extendoActuator.setTargetPosition(0);
//        }
        robot.extendoActuator.periodic();
//        robot.intakeArmPivotActuator.periodic();
//        robot.intakeClawPivotActuator.periodic();
    }


    @Override
    public void read() {
        robot.extendoActuator.read();
//        robot.intakeArmPivotActuator.read();
//        robot.intakeClawPivotActuator.read();
    }

    @Override
    public void write() {
        robot.extendoActuator.write();
//        robot.intakeArmPivotActuator.write();
//        robot.intakeClawPivotActuator.write();
    }

    @Override
    public void reset() {
        robot.extendoActuator.setTargetPosition(0);
        setPivotState(PivotState.TRANSFER);
        setClawState(ClawState.OPEN);
        setClawRotation(ClawRotationState.TRANSFER);
    }
}
