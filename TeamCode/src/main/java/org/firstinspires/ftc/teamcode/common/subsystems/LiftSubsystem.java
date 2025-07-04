package org.firstinspires.ftc.teamcode.common.subsystems;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class LiftSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public LiftState liftState = LiftState.RETRACTED;
    public ClawState clawState = ClawState.OPEN;

    public boolean huggingSpecDeposit = false;
    public boolean isResetting = false;
    private int liftTargetTicks = 0;

    public enum ClawState {
        OPEN(DEPOSIT_CLAW_OPEN_POS),
        OPEN_TRANSFER(DEPOSIT_CLAW_OPEN_TRANSFER_POS),
        MICRO_OPEN(DEPOSIT_CLAW_MICRO_OPEN_POS),
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
        INTAKE_SPECIMEN,
        HOLDING_SPECIMEN,
        DEPOSIT_LOW_SPECIMEN,
        DEPOSIT_HIGH_RUNG_SETUP,
        DEPOSIT_HIGH_SPECIMEN,
        PUSHING_SPECIMEN,
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
        robot.depositArmPivotBottomServo.setPosition(getArmPivotPosition(liftState));
        robot.depositArmPivotTopServo.setPosition(getArmPivotPosition(liftState));
        //robot.depositArmPivotActuator.setTargetPosition(getArmPivotPosition(liftState));
        robot.depositClawPivotServo.setPosition(getClawPivotPosition(liftState));
        //robot.depositArmPivotTopServo.setPosition(getArmPivotPosition(liftState));
        robot.depositClawRotationServo.setPosition(getClawRotationPosition(liftState));
        robot.liftActuator.setTargetPosition(getActuatorPosition(liftState));
    }

    public void updateState(@NotNull LiftState state, double liftHeight) {
        this.liftState = state;
        robot.depositArmPivotBottomServo.setPosition(getArmPivotPosition(liftState));
        robot.depositArmPivotTopServo.setPosition(getArmPivotPosition(liftState));
        //robot.depositArmPivotActuator.setTargetPosition(getArmPivotPosition(liftState));
        robot.depositClawPivotServo.setPosition(getClawPivotPosition(liftState));
        //robot.depositArmPivotTopServo.setPosition(getArmPivotPosition(liftState));
        robot.depositClawRotationServo.setPosition(getClawRotationPosition(liftState));
        if(liftHeight == 0.0); // do nothing
        else robot.liftActuator.setTargetPosition(liftHeight);
    }

    /**
     * Updates the claw state and sets the servo positions.
     *
     * @param state The desired claw state.
     */
    public void setClawState(@NotNull ClawState state) {
        this.clawState = state;
        robot.depositClawServo.setPosition(state.getPosition());
    }

    public void setLiftTargetPosTicks(int pos){
        this.liftTargetTicks = (int) MathUtils.clamp(pos, 0, MAX_SLIDES_EXTENSION);
        robot.liftActuator.setTargetPosition(this.liftTargetTicks);
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
            case RETRACTED:
            case INTAKE_SPECIMEN:
                return 0;
            case HOLDING_SPECIMEN:
                return HOLDING_SPECIMEN_HEIGHT;
            case DEPOSIT_LOW_SPECIMEN: return LOW_SPECIMEN_HEIGHT;
            case DEPOSIT_HIGH_RUNG_SETUP:
                if (IS_AUTONOMOUS){
                    if (huggingSpecDeposit) return HIGH_SPECIMEN_HUGGING_SETUP_HEIGHT;
                    else return HIGH_SPECIMEN_SETUP_AUTONOMOUS_HEIGHT;
                }
                else return HIGH_SPECIMEN_SETUP_TELEOP_HEIGHT;
            case DEPOSIT_HIGH_SPECIMEN:
                if (IS_AUTONOMOUS){
                    if (huggingSpecDeposit) return HIGH_SPECIMEN_HUGGING_HEIGHT;
                    else return HIGH_SPECIMEN_AUTONOMOUS_HEIGHT;
                }
                else return HIGH_SPECIMEN_TELEOP_HEIGHT;
            case PUSHING_SPECIMEN: return PUSHING_SPECIMEN_HEIGHT;
            case DEPOSIT_LOW_BUCKET: return LOW_BUCKET_HEIGHT;
            case DEPOSIT_HIGH_BUCKET:
                if (IS_AUTONOMOUS) return HIGH_BUCKET__AUTO_HEIGHT;
                else return HIGH_BUCKET_HEIGHT;
            case LVL1_ASCENT: return LVL1_ASCENT_HEIGHT;
            case LVL2_ASCENT_SETUP: return ENDGAME_ASCENT_SETUP_HEIGHT;
            case LVL2_ASCENT_DOWN: return ENDGAME_ASCENT_HEIGHT;
            default: throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }


    // TODO: add in claw pivot positions
    /**
     * Retrieves the claw pivot servo position for the given lift state.
     */
    private double getClawPivotPosition(LiftState state) {
        switch (state) {
            case RETRACTED:
            case LVL2_ASCENT_SETUP:
            case LVL2_ASCENT_DOWN:
                return DEPOSIT_CLAW_PIVOT_TRANSFER_POS;
            case LVL1_ASCENT:
                return DEPOSIT_CLAW_PIVOT_LVL1_ASCENT_POS;
            case INTAKE_SPECIMEN:
            case HOLDING_SPECIMEN:
                return DEPOSIT_CLAW_PIVOT_SPECIMEN_INTAKE_POS;
            case DEPOSIT_HIGH_RUNG_SETUP:
                if (IS_AUTONOMOUS){
                    if (huggingSpecDeposit) return DEPOSIT_CLAW_PIVOT_HUGGING_SPECIMEN_SCORING_SETUP_POS;
                    return DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_SETUP_AUTONOMOUS_POS;
                }
                else return DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_SETUP_TELEOP_POS;
            case DEPOSIT_LOW_SPECIMEN:
            case DEPOSIT_HIGH_SPECIMEN:
                if (IS_AUTONOMOUS){
                    if (huggingSpecDeposit) return DEPOSIT_CLAW_PIVOT_HUGGING_SPECIMEN_SCORING_POS;
                    return DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_AUTONOMOUS_POS;
                }
                else return DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_TELEOP_POS;
            case PUSHING_SPECIMEN: return DEPOSIT_CLAW_PIVOT_PUSHING_SPECIMEN_POS;
            case DEPOSIT_LOW_BUCKET:
            case DEPOSIT_HIGH_BUCKET:
                if (IS_AUTONOMOUS) return DEPOSIT_CLAW_PIVOT_AUTONOMOUS_BUCKET_POS;
                else return DEPOSIT_CLAW_PIVOT_BUCKET_POS;
            default: throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }

    /**
     * Retrieves the arm pivot servo position for the given lift state.
     */
    private double getArmPivotPosition(LiftState state) {
        switch (state) {
            case RETRACTED:
            case LVL2_ASCENT_SETUP:
            case LVL2_ASCENT_DOWN:
                return DEPOSIT_ARM_PIVOT_TRANSFER_POS;
            case LVL1_ASCENT:
                return DEPOSIT_ARM_PIVOT_LVL1_ASCENT_POS;
            case INTAKE_SPECIMEN:
            case HOLDING_SPECIMEN:
                return DEPOSIT_ARM_PIVOT_SPECIMEN_INTAKE_POS;
            case DEPOSIT_LOW_SPECIMEN:
            case DEPOSIT_HIGH_SPECIMEN:
            case DEPOSIT_HIGH_RUNG_SETUP:
                if (IS_AUTONOMOUS){
                    if (huggingSpecDeposit) return DEPOSIT_ARM_PIVOT_HUGGING_SPECIMEN_SCORING_POS;
                    else return DEPOSIT_ARM_PIVOT_SPECIMEN_SCORING_AUTONOMOUS_POS;
                }
                else return DEPOSIT_ARM_PIVOT_SPECIMEN_SCORING_TELEOP_POS;
            case PUSHING_SPECIMEN: return DEPOSIT_ARM_PIVOT_PUSHING_SPECIMEN_POS;
            case DEPOSIT_LOW_BUCKET:
            case DEPOSIT_HIGH_BUCKET:
                if (IS_AUTONOMOUS) return DEPOSIT_ARM_PIVOT_AUTONOMOUS_BUCKET_POS;
                else return DEPOSIT_ARM_PIVOT_BUCKET_POS;
            default: throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }

    /**
     * Retrieves the claw rotation servo position for the given lift state.
     */
    private double getClawRotationPosition(LiftState state) {
        switch (state) {
            case RETRACTED:
            case LVL1_ASCENT:
            case LVL2_ASCENT_SETUP:
            case LVL2_ASCENT_DOWN:
                return DEPOSIT_CLAW_ROTATION_TRANSFER_POS;
            case HOLDING_SPECIMEN:
            case INTAKE_SPECIMEN:
                if (IS_AUTONOMOUS) return DEPOSIT_CLAW_ROTATION_TRANSFER_POS;
                else return DEPOSIT_CLAW_ROTATION_SAMPLE_OZ_DROP_TELEOP_POS;
            case DEPOSIT_LOW_SPECIMEN:
            case DEPOSIT_HIGH_RUNG_SETUP:
            case DEPOSIT_HIGH_SPECIMEN:
            case PUSHING_SPECIMEN:
                if (IS_AUTONOMOUS){
                    return DEPOSIT_CLAW_ROTATION_SPECIMEN_SCORING_AUTONOMOUS_POS;
                }
                else return DEPOSIT_CLAW_ROTATION_SPECIMEN_SCORING_POS;
            case DEPOSIT_LOW_BUCKET:
                return DEPOSIT_CLAW_ROTATION_LOW_BUCKET_SCORING_POS;
            case DEPOSIT_HIGH_BUCKET:
                return IS_AUTONOMOUS ? DEPOSIT_CLAW_ROTATION_AUTO_BUCKET_SCORING_POS : DEPOSIT_CLAW_ROTATION_TELEOP_BUCKET_SCORING_POS;

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
        setClawState(ClawState.OPEN);
    }
}