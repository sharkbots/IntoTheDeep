package org.firstinspires.ftc.teamcode.common.commandbase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class HoldPointCommand extends CommandBase {
    private final Follower follower;
    private Pose point;
    private final Robot robot;
    boolean dynMode = false;
    private DynBuilder dynPoseBuilder;
    public interface DynBuilder{
        Pose run();
    }

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
        if (dynMode) point = dynPoseBuilder.run();
        Globals.IS_DT_AUTO_ALIGNING = true;
        robot.telemetryA.addData("target pose:", point.toString());
        robot.telemetryA.update();
        follower.holdPoint(point);

        if (dynMode) point = null;
    }

    @Override
    public boolean isFinished() {
        if (!follower.isBusy()){
            Globals.IS_DT_AUTO_ALIGNING = false;
            //follower.breakFollowing();
            return true;
        }
        else return false;
    }
}
