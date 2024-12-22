package org.firstinspires.ftc.teamcode.common.subsystems;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public PivotState pivotState = PivotState.TRANSFER;
    public ClawState clawState = ClawState.OPEN;
    public ClawRotationState clawRotationState = ClawRotationState.TRANSFER;

    public int extendoTargetPos = 0;
    private final int MAX_EXTENDO_EXTENSION = 1890;

    public enum PivotState{
        HOVERING,
        INTAKING,
        TRANSFER
    }

    public enum ClawRotationState{
        TRANSFER(0.54),
        FULLY_LEFT(0.88),
        LEFT_30_DEGREES(TRANSFER.getPosition()+1*(FULLY_LEFT.getPosition()-TRANSFER.getPosition())/3),
        LEFT_60_DEGREES(TRANSFER.getPosition()+2*(FULLY_LEFT.getPosition()-TRANSFER.getPosition())/3),
        FULLY_RIGHT(0.195),
        RIGHT_30_DEGREES(TRANSFER.getPosition()+1*(FULLY_RIGHT.getPosition()-TRANSFER.getPosition())/3),
        RIGHT_60_DEGREES(TRANSFER.getPosition()+2*(FULLY_RIGHT.getPosition()-TRANSFER.getPosition())/3);

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
        reset();
    }

    public void moveLeft() {
        switch (clawRotationState) {
            case FULLY_RIGHT:
                clawRotationState = ClawRotationState.RIGHT_60_DEGREES;
                setClawRotation(ClawRotationState.RIGHT_60_DEGREES);
                break;
            case RIGHT_60_DEGREES:
                clawRotationState = ClawRotationState.RIGHT_30_DEGREES;
                setClawRotation(ClawRotationState.RIGHT_30_DEGREES);
                break;
            case RIGHT_30_DEGREES:
                clawRotationState = ClawRotationState.TRANSFER;
                setClawRotation(ClawRotationState.TRANSFER);
                break;
            case TRANSFER:
                clawRotationState = ClawRotationState.LEFT_30_DEGREES;
                setClawRotation(ClawRotationState.LEFT_30_DEGREES);
                break;
            case LEFT_30_DEGREES:
                clawRotationState = ClawRotationState.LEFT_60_DEGREES;
                setClawRotation(ClawRotationState.LEFT_60_DEGREES);
                break;
            case LEFT_60_DEGREES:
                clawRotationState = ClawRotationState.FULLY_LEFT;
                setClawRotation(ClawRotationState.FULLY_LEFT);
                break;
            case FULLY_LEFT:
                // Do nothing if already at the extreme
                break;
            default:
                break;
        }
        //setClawRotation(clawRotationState);
    }

    public void moveRight() {
        switch (clawRotationState) {
            case FULLY_LEFT:
                clawRotationState = ClawRotationState.LEFT_60_DEGREES;
                setClawRotation(ClawRotationState.LEFT_60_DEGREES);
                break;
            case LEFT_60_DEGREES:
                clawRotationState = ClawRotationState.LEFT_30_DEGREES;
                setClawRotation(ClawRotationState.LEFT_30_DEGREES);
                break;
            case LEFT_30_DEGREES:
                clawRotationState = ClawRotationState.TRANSFER;
                setClawRotation(ClawRotationState.TRANSFER);
                break;
            case TRANSFER:
                clawRotationState = ClawRotationState.RIGHT_30_DEGREES;
                setClawRotation(ClawRotationState.RIGHT_30_DEGREES);
                break;
            case RIGHT_30_DEGREES:
                clawRotationState = ClawRotationState.RIGHT_60_DEGREES;
                setClawRotation(ClawRotationState.RIGHT_60_DEGREES);
                break;
            case RIGHT_60_DEGREES:
                clawRotationState = ClawRotationState.FULLY_RIGHT;
                setClawRotation(ClawRotationState.FULLY_RIGHT);
                break;
            case FULLY_RIGHT:
                // Do nothing if already at the extreme
                break;
            default:
                break;
        }
        //setClawRotation(clawRotationState);
    }

    public void setExtendoTarget(int pos){
        this.extendoTargetPos = Math.max(Math.min(pos, MAX_EXTENDO_EXTENSION), 0);
        robot.extendoActuator.setTargetPosition(this.extendoTargetPos);
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
        this.clawRotationState = state;
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
                return 0.74+0.015;
            case INTAKING:
                return 0.79+0.015;
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
