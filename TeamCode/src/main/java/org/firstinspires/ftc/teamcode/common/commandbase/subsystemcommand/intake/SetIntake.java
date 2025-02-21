package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class SetIntake extends CommandBase {
    private final Robot robot;
    private final IntakeSubsystem.PivotState pivotState;
    private final Double clawTargetRotationDegrees; // Optional rotation target

    ElapsedTime timer;
    private double prevClawPivotPos, prevArmPivotPos, prevClawRotationPos;
    private double currClawPivotPos, currArmPivotPos, currClawRotationPos;

    // Constructor without target rotation (uses pivot state for claw rotation)
    public SetIntake(Robot robot, IntakeSubsystem.PivotState pivotState) {
        this(robot, pivotState, null);
    }

    // Constructor with target rotation (overloaded)
    public SetIntake(Robot robot, IntakeSubsystem.PivotState pivotState, Double clawTargetRotationDegrees) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.clawTargetRotationDegrees = clawTargetRotationDegrees; // Can be null
        this.timer = new ElapsedTime();
    }


    @Override
    public void initialize(){
        // Update pivot and cache positions for timing
        prevClawPivotPos = robot.intakeClawPivotServo.getPosition();
        prevArmPivotPos = robot.intakeArmPivotLeftServo.getPosition();
        prevClawRotationPos = robot.intakeClawRotationServo.getPosition();

        // Set the pivot state for the arm and claw
        robot.intake.setPivotState(pivotState);

        // If targetRotationDegrees is provided, set the claw rotation to the specified angle
        if (clawTargetRotationDegrees != null) {
            robot.intake.setClawRotationDegrees(clawTargetRotationDegrees);
        } else {
            // Otherwise, use the pivot state to set the claw rotation
            robot.intake.setClawRotation(pivotState);
        }

        currClawPivotPos = robot.intakeClawPivotServo.getPosition();
        currArmPivotPos = robot.intakeArmPivotLeftServo.getPosition();
        currClawRotationPos = robot.intakeClawRotationServo.getPosition();

        timer.reset();
    }

    @Override
    public boolean isFinished(){
        return (timer.milliseconds() > Math.abs(prevClawPivotPos - currClawPivotPos) * INTAKE_CLAW_PIVOT_MOVEMENT_TIME)
                && (timer.milliseconds() > Math.abs(prevArmPivotPos - currArmPivotPos) * INTAKE_ARM_PIVOT_MOVEMENT_TIME)
                && (timer.milliseconds() > Math.abs(prevClawRotationPos - currClawRotationPos) * INTAKE_CLAW_ROTATION_MOVEMENT_TIME);
    }
}
