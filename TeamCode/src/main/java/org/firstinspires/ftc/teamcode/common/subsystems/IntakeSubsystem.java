package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class IntakeSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public PivotState pivotState = PivotState.TRANSFER;
    public ClawState clawState = ClawState.OPEN;
    //public ClawRotationState clawRotationState = ClawRotationState.TRANSFER;

    private double clawRotationAngleDegrees = 0;

    public int extendoTargetPos = 0;

    public enum PivotState{
        HOVERING_NO_SAMPLE,
        HOVERING_NO_SAMPLE_MANUAL,
        HOVERING_WITH_SAMPLE,
        INTAKE,
        TRANSFER
    }

    public enum ClawState{
        OPEN(INTAKE_CLAW_OPEN_POS),
        MICRO_OPEN(INTAKE_CLAW_MICRO_OPEN_POS),
        CLOSED(INTAKE_CLAW_CLOSED_POS);

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

    public void setExtendoTargetTicks(int pos){
        this.extendoTargetPos = Math.max(Math.min(pos, MAX_EXTENDO_EXTENSION), 0);
        if (extendoTargetPos > EXTENDO_FEEDFORWARD_TRIGGER_THRESHOLD){
            if (getExtendoPosTicks() < extendoTargetPos) robot.extendoActuator.updateFeedforward(EXTENDO_FEEDFORWARD_EXTENDING);
            else robot.extendoActuator.updateFeedforward(EXTENDO_FEEDFORWARD_RETRACTING);
        }
        else robot.extendoActuator.updateFeedforward(0);
        robot.extendoActuator.setTargetPosition(this.extendoTargetPos);
    }

    public void setExtendoTargetInches(double inches){
        setExtendoTargetTicks((int)(inches * EXTENDO_TICKS_PER_INCH));
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

    public void setClawRotationDegrees(double targetAngleDegrees){
        // Clamp the angle between -90 and +90 degrees
        targetAngleDegrees = Math.max(-90, Math.min(90, targetAngleDegrees));
        this.clawRotationAngleDegrees = targetAngleDegrees;


        // Calculate normalized value (0.0 to 1.0) across the angle range
        double normalized = (targetAngleDegrees + 90) / 180.0;

        // Interpolate between the left and right positions
        double servoPosition = INTAKE_CLAW_ROTATION_FULLY_LEFT_POS
                + normalized * (INTAKE_CLAW_ROTATION_FULLY_RIGHT_POS - INTAKE_CLAW_ROTATION_FULLY_LEFT_POS);

        // Set the servo position
        robot.intakeClawRotationServo.setPosition(servoPosition);
    }

    public void setClawRotation(@NotNull PivotState state) {
        double servoPosition = getClawRotationPosition(state);
        robot.intakeClawRotationServo.setPosition(getClawRotationPosition(state));

        // Update clawRotationAngleDegrees based on the servo position
        this.clawRotationAngleDegrees = getClawRotationAngleFromPosition(servoPosition);
    }

    /**
     * Converts a servo position to an angle in degrees.
     */
    private double getClawRotationAngleFromPosition(double servoPosition) {
        // Normalize the servo position to a 0.0-1.0 range relative to the left and right positions
        double normalized = (servoPosition - INTAKE_CLAW_ROTATION_FULLY_LEFT_POS)
                / (INTAKE_CLAW_ROTATION_FULLY_RIGHT_POS - INTAKE_CLAW_ROTATION_FULLY_LEFT_POS);

        // Convert the normalized value to an angle in degrees (-90 to +90)
        return normalized * 180.0 - 90;
    }

    public double getClawRotationDegrees(){
        return this.clawRotationAngleDegrees;
    }

    public double getExtendoPosTicks(){
        return robot.extendoActuator.getPosition();
    }

    public double getExtendoPosInches(){
        return getExtendoPosTicks()/EXTENDO_TICKS_PER_INCH;
    }

    /**
     * Retrieves the claw rotation servo position for the given pivot state.
     */
    private double getClawRotationPosition(PivotState state) {
        switch (state) {
            case TRANSFER:
            case HOVERING_NO_SAMPLE:
            case HOVERING_NO_SAMPLE_MANUAL:
            case HOVERING_WITH_SAMPLE:
            case INTAKE:
                return INTAKE_CLAW_ROTATION_TRANSFER_POS;
            default:
                throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

    /**
     * Retrieves the claw pivot servo position for the given pivot state.
     */
    private double getClawPivotPosition(PivotState state) {
        switch (state) {
            case TRANSFER:
                return INTAKE_CLAW_PIVOT_TRANSFER_POS;
            case HOVERING_NO_SAMPLE:
                return INTAKE_CLAW_PIVOT_HOVER_INTAKE_POS;
            case HOVERING_NO_SAMPLE_MANUAL:
                return INTAKE_CLAW_PIVOT_HOVER_INTAKE_MANUAL_POS;
            case HOVERING_WITH_SAMPLE:
                return INTAKE_CLAW_PIVOT_HOLDING_POS;
            case INTAKE:
                return INTAKE_CLAW_PIVOT_INTAKE_POS;
            default: throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

    /**
     * Retrieves the arm pivot servo position for the given pivot state.
     */
    private double getArmPivotPosition(PivotState state) {
        switch (state) {
            case TRANSFER:
                return INTAKE_ARM_PIVOT_TRANSFER_POS;
            case HOVERING_NO_SAMPLE:
                return INTAKE_ARM_PIVOT_HOVER_INTAKE_POS;
            case HOVERING_NO_SAMPLE_MANUAL:
                return INTAKE_ARM_PIVOT_HOVER_INTAKE_MANUAL_POS;
            case HOVERING_WITH_SAMPLE:
                return INTAKE_ARM_PIVOT_HOVER_WITH_SAMPLE_POS;
            case INTAKE:
                return INTAKE_ARM_PIVOT_INTAKE_POS;
            default: throw new IllegalArgumentException("Unknown PivotState: " + state);
        }
    }

    public boolean pivotReached(){
        return robot.intakeArmPivotActuator.hasReached();
    }

    public boolean extendoReached(){
        return robot.extendoActuator.hasReached();
    }

    @Override
    public void periodic() {
        robot.extendoActuator.periodic();
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
        setClawRotation(PivotState.TRANSFER);
        setClawState(ClawState.OPEN);
        //robot.intakeClawRotationServo.setPosition(INTAKE_CLAW_ROTATION_TRANSFER_POS);
    }
}