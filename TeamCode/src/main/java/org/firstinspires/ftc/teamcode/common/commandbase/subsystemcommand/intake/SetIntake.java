package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

public class SetIntake extends CommandBase {
    private final Robot robot;
    private final IntakeSubsystem.PivotState pivotState;

    ElapsedTime timer;
    private double prevClawPivotPos;
    private double currClawPivotPos;
    private double prevArmPivotPos;
    private double currArmPivotPos;

    public SetIntake(Robot robot, IntakeSubsystem.PivotState pivotState) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.timer = new ElapsedTime();
    }

    @Override
    public void initialize(){
        // Update pivot and cache positions for timing
        prevClawPivotPos = robot.intakeClawPivotServo.getPosition();
        prevArmPivotPos = robot.intakeArmPivotLeftServo.getPosition();
        robot.intake.setPivotState(pivotState);
        currClawPivotPos = robot.intakeClawPivotServo.getPosition();
        currArmPivotPos = robot.intakeArmPivotLeftServo.getPosition();

        timer.reset();
    }

    @Override
    public boolean isFinished(){
        return (timer.milliseconds() > Math.abs(prevClawPivotPos - currClawPivotPos) * INTAKE_CLAW_PIVOT_MOVEMENT_TIME)
                && (timer.milliseconds() > Math.abs(prevArmPivotPos - currArmPivotPos) * INTAKE_ARM_PIVOT_MOVEMENT_TIME);
    }
}
