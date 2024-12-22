package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.jetbrains.annotations.NotNull;

public class LiftSubsystem extends SubsystemWrapper {

    private final Robot robot;

    public LiftState liftState = LiftState.RETRACTED;
    public ClawState clawState = ClawState.OPEN;

    public enum ClawState {
        OPEN(0.65),
        MICRO_OPEN(0.71),
        CLOSED(0.9);

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
        INTAKE_WALL,
        DEPOSIT_LOW_RUNG,
        DEPOSIT_HIGH_RUNG_SETUP,
        DEPOSIT_HIGH_RUNG_DOWN,
        DEPOSIT_LOW_BASKET,
        DEPOSIT_HIGH_BASKET,
        LOW_HANG
    }

    public LiftSubsystem() {
        this.robot = Robot.getInstance();
        updateState(liftState);
        updateState(ClawState.MICRO_OPEN);
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
        return liftState == LiftState.INTAKE_WALL
                || liftState == LiftState.DEPOSIT_LOW_RUNG
                || liftState == LiftState.DEPOSIT_HIGH_RUNG_SETUP
                || liftState == LiftState.DEPOSIT_LOW_BASKET
                || liftState == LiftState.DEPOSIT_HIGH_BASKET;
    }

    /**
     * Retrieves the actuator position for the given lift state.
     */
    private int getActuatorPosition(LiftState state) {
        switch (state) {
            case TRANSFER:
            case RETRACTED:
            case INTAKE_WALL:
                return 0;
            case DEPOSIT_LOW_RUNG: return 85;
            case DEPOSIT_HIGH_RUNG_SETUP: return 785;
            case DEPOSIT_HIGH_RUNG_DOWN: return 620;
            case DEPOSIT_LOW_BASKET: return 925;
            case DEPOSIT_HIGH_BASKET: return 1925;
            case LOW_HANG: return 600;
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
            case LOW_HANG:
                return 0.045;
            case INTAKE_WALL: return 0.96;
            case DEPOSIT_LOW_RUNG:
            case DEPOSIT_HIGH_RUNG_DOWN:
            case DEPOSIT_HIGH_RUNG_SETUP:
                return 0.88;
            case DEPOSIT_LOW_BASKET:
            case DEPOSIT_HIGH_BASKET:
                return 0.81;
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
            case LOW_HANG:
            case INTAKE_WALL:
            case DEPOSIT_LOW_RUNG:
            case DEPOSIT_HIGH_RUNG_SETUP:
            case DEPOSIT_HIGH_RUNG_DOWN:
                return 0.35;
            case DEPOSIT_LOW_BASKET:
            case DEPOSIT_HIGH_BASKET:
                return Globals.ALLIANCE == Globals.ALLIANCE.RED ? 0.02 : 0.685;
            default:
                throw new IllegalArgumentException("Unknown LiftState: " + state);
        }
    }


    @Override
    public void periodic() {
        robot.liftActuator.setTargetPosition(getActuatorPosition(liftState));
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
        robot.liftActuator.setTargetPosition(0);
        this.liftState = LiftState.RETRACTED;
        updateState(LiftState.RETRACTED);
    }
}