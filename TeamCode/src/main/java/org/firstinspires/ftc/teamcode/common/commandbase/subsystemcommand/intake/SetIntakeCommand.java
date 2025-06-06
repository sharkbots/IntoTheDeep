package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class SetIntakeCommand extends CommandBase {
    private final Robot robot;
    private final IntakeSubsystem.PivotState pivotState;
    private Double clawTargetRotationDegrees; // Optional rotation target

    ElapsedTime timer;
    private double prevClawPivotPos, prevArmPivotPos, prevClawRotationPos;
    private double currClawPivotPos, currArmPivotPos, currClawRotationPos;

    private DynBuilder dynDoubleBuilder;
    boolean dynMode = false;
    public interface DynBuilder{
        Double run();
    }

    // Constructor without target rotation (uses pivot state for claw rotation)
    public SetIntakeCommand(Robot robot, IntakeSubsystem.PivotState pivotState) {
        this(robot, pivotState, (Double) null);
    }

    // Constructor with target rotation (overloaded)
    public SetIntakeCommand(Robot robot, IntakeSubsystem.PivotState pivotState, Double clawTargetRotationDegrees) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.clawTargetRotationDegrees = clawTargetRotationDegrees; // Can be null
        this.timer = new ElapsedTime();
    }

    public SetIntakeCommand(Robot robot, IntakeSubsystem.PivotState pivotState, DynBuilder dynDoubleBuilder) {
        this(robot, pivotState, (Double) null);
        dynMode = true;
        this.dynDoubleBuilder = dynDoubleBuilder;
    }


    @Override
    public void initialize(){
        if (dynMode) clawTargetRotationDegrees = dynDoubleBuilder.run();
        // Update pivot and cache positions for timing
        prevClawPivotPos = robot.intakeClawPivotServo.getPosition();
        prevArmPivotPos = robot.intakeArmPivotLeftServo.getPosition();
        prevClawRotationPos = robot.intakeClawRotationServo.getPosition();

        // If targetRotationDegrees is provided, set the claw rotation to the specified angle
        if (clawTargetRotationDegrees != null) {
            if (clawTargetRotationDegrees == 0.0){
                robot.intake.setClawRotation(pivotState);
            }
            else {
                robot.intake.setClawRotationDegrees(clawTargetRotationDegrees);
            }
        } else {
            // do nothing
        }

        // Set the pivot state for the arm and claw
        robot.intake.setPivotState(pivotState);

        if (dynMode) clawTargetRotationDegrees = null;

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
