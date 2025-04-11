package org.firstinspires.ftc.teamcode.common.commandbase;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;

public class HoldPointCommand extends CommandBase {
    private final Follower follower;
    private Pose point;
    private final Robot robot;
    boolean dynMode = false;
    private DynBuilder dynPoseBuilder;
    private double targetMagnitude;

    public interface DynBuilder{
        Pose run();
    }
    ElapsedTime timer;
    double timeout;

    public HoldPointCommand(Follower follower, Pose point) {
        this.follower = follower;
        this.point = point;
        robot = Robot.getInstance();
    }

    public HoldPointCommand(Follower follower, DynBuilder dynPoseBuilder){
        this(follower, (Pose) null);
        this.dynPoseBuilder = dynPoseBuilder;
        dynMode = true;
    }


    @Override
    public void initialize() {
        follower.breakFollowing();
        if (dynMode) point = dynPoseBuilder.run();
        Globals.IS_DT_AUTO_ALIGNING = true;
        robot.telemetryA.addData("target pose:", point.toString());
        robot.telemetryA.update();

        Pose robotPose = robot.follower.getPose();
        Vector targetVector = MathFunctions.subtractVectors(point.getVector(), robotPose.getVector());
        targetMagnitude = targetVector.getMagnitude();
        Vector ffTargetVector = new Vector(targetMagnitude+Globals.HOLDPOINT_MANUAL_FEEDFORWARD, targetVector.getTheta());
        robotPose.add(new Pose(ffTargetVector.getXComponent(), ffTargetVector.getYComponent(), 0));

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2, 0, 0.01, 0);
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.3605;



        follower.holdPoint(robotPose);

        robot.telemetryA.addData("target magnitude", targetMagnitude);
        robot.telemetryA.update();
        //timeout = Math.min(750, MathFunctions.subtractVectors(robot.follower.getPose().getVector(), point.getVector()).getMagnitude() * );

        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public boolean isFinished() {
        // TODO: add timeout in globals
        timeout = MathUtils.clamp(targetMagnitude/3.3 * 1500, 250, 1500);
        if (timer.milliseconds() > timeout){
            Globals.IS_DT_AUTO_ALIGNING = false;
            if (dynMode) point = null;

//            follower.setTranslationalPIDF();
//            follower.setSecondaryTranslationalPIDF();
//            follower.setHeadingPIDF();
//            follower.setSecondaryHeadingPIDF();

            return true;
        }
        else {
            return false;
        }
    }
}
