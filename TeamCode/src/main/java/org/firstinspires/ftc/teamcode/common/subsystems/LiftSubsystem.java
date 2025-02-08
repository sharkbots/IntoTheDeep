package org.firstinspires.ftc.teamcode.common.subsystems;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class LiftSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public LiftState liftState = LiftState.RETRACTED;
    public ClawState clawState = ClawState.OPEN;

    public enum ClawState {
        OPEN(DEPOSIT_CLAW_OPEN_POS),
        CLOSED(DEPOSIT_CLAW_CLOSED_POS);

        private final double position;

        ClawState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum LiftState {
        RETRACTED,
        TRANSFER,
        INTAKE_SPECIMEN,
        HOLDING_SPECIMEN,
        DEPOSIT_LOW_SPECIMEN,
        DEPOSIT_HIGH_RUNG_SETUP,
        DEPOSIT_HIGH_SPECIMEN,
        DEPOSIT_LOW_BUCKET,
        DEPOSIT_HIGH_BUCKET,
        LVL1_ASCENT,
        LVL2_ASCENT_SETUP,
        LVL2_ASCENT_DOWN;
    }

    public LiftSubsystem() {
        this.robot = Robot.getInstance();
    }

    /**
     * Updates the lift state and sets the servo positions.
     *
     * @param state The desired lift state.
     */
    public void updateState(@NotNull LiftState state) {
        this.liftState = state;

        robot.depositPivotServo.setPosition(getClawPivotPosition(liftState));
        robot.depositClawRotationServo.setPosition(getClawRotationPosition(liftState));
        robot.liftActuator.setTargetPosition(getActuatorPosition(liftState));
    }

    /**
     * Updates the claw state and sets the servo positions.
     *
     * @param state The desired claw state.
     */
    public void updateState(@NotNull ClawState state) {
        this.clawState = state;
        robot.depositClawServo.setPosition(state.getPosition());
    }

    public LiftState getLiftState() {
        return liftState;
    }

    public boolean isActuatorAtTarget() {
        return robot.liftActuator.hasReached();
    }

    public boolean isClawControlAllowed() {
        return liftState == LiftState.DEPOSIT_LOW_SPECIMEN
                || liftState == LiftState.DEPOSIT_HIGH_RUNG_SETUP
                || liftState == LiftState.DEPOSIT_LOW_BUCKET
                || liftState == LiftState.DEPOSIT_HIGH_BUCKET;
    }

    /**
     * Retrieves the actuator position for the given lift state.
     */
    private int getActuatorPosition(LiftState state) {
        switch (state) {
            case TRANSFER:
            case RETRACTED:
            case INTAKE_SPECIMEN:
                return 0;
            case HOLDING_SPECIMEN:
                return HOLDING_SPECIMEN_HEIGHT;
            case DEPOSIT_LOW_SPECIMEN: return LOW_SPECIMEN_HEIGHT;
            case DEPOSIT_HIGH_RUNG_SETUP: return HIGH_SPECIMEN_SETUP_HEIGHT;
            case DEPOSIT_HIGH_SPECIMEN: return HIGH_SPECIMEN_HEIGHT;
            case DEPOSIT_LOW_BUCKET: return LOW_BUCKET_HEIGHT;
            case DEPOSIT_HIGH_BUCKET: return HIGH_BUCKET_HEIGHT;
            case LVL1_ASCENT: return LVL1_ASCENT_HEIGHT;
            case LVL2_ASCENT_SETUP: return ENDGAME_ASCENT_SETUP_HEIGHT;
            case LVL2_ASCENT_DOWN: return ENDGAME_ASCENT_HEIGHT;
            default: throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }

    /**
     * Retrieves the claw pivot servo position for the given lift state.
     */
    private double getClawPivotPosition(LiftState state) {
        switch (state) {
            case TRANSFER:
            case RETRACTED:
            case LVL1_ASCENT:
            case LVL2_ASCENT_SETUP:
            case LVL2_ASCENT_DOWN:
                return DEPOSIT_CLAW_PIVOT_TRANSFER_POS;
            case INTAKE_SPECIMEN: return DEPOSIT_CLAW_PIVOT_SPECIMEN_INTAKE_POS;
            case DEPOSIT_LOW_SPECIMEN:
            case DEPOSIT_HIGH_SPECIMEN:
            case DEPOSIT_HIGH_RUNG_SETUP:
            case HOLDING_SPECIMEN:
                return DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_POS;
            case DEPOSIT_LOW_BUCKET:
            case DEPOSIT_HIGH_BUCKET:
                return DEPOSIT_CLAW_PIVOT_BUCKET_POS;
            default: throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }

    /**
     * Retrieves the claw rotation servo position for the given lift state.
     */
    private double getClawRotationPosition(LiftState state) {
        switch (state) {
            case TRANSFER:
            case RETRACTED:
            case LVL1_ASCENT:
            case INTAKE_SPECIMEN:
            case DEPOSIT_LOW_SPECIMEN:
            case DEPOSIT_HIGH_RUNG_SETUP:
            case DEPOSIT_HIGH_SPECIMEN:
            case HOLDING_SPECIMEN:
            case LVL2_ASCENT_SETUP:
            case LVL2_ASCENT_DOWN:
                return DEPOSIT_CLAW_ROTATION_TRANSFER_POS;
            case DEPOSIT_LOW_BUCKET:
            case DEPOSIT_HIGH_BUCKET:
                return Globals.ALLIANCE == Globals.AllianceColor.RED ?
                        DEPOSIT_CLAW_ROTATION_BUCKET_SCORING_RED_POS : DEPOSIT_CLAW_ROTATION_BUCKET_SCORING_BLUE_POS;

            default:
                throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }

    public boolean liftReached(){
        return robot.liftActuator.hasReached();
    }

    @Override
    public void periodic() {
        robot.liftActuator.periodic();
    }

    @Override
    public void read() {
        robot.liftActuator.read();
    }

    @Override
    public void write() {
        robot.liftActuator.write();
    }

    @Override
    public void reset() {
        robot.liftActuator.updateFeedforward(0);
        robot.liftActuator.setTargetPosition(0);
        this.liftState = LiftState.RETRACTED;
        updateState(LiftState.RETRACTED);
        updateState(ClawState.OPEN);
    }
}