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
    private final Pose point;
    private final Robot robot;

    public HoldPointCommand(Follower follower, Pose point) {
        this.follower = follower;
        this.point = point;
        robot = Robot.getInstance();
    }


    @Override
    public void initialize() {
        Globals.IS_DT_AUTO_ALIGNING = true;
        follower.holdPoint(point);
    }

    @Override
    public boolean isFinished() {
        if (!follower.isBusy()){
            Globals.IS_DT_AUTO_ALIGNING = false;
            return true;
        }
        else return false;
    }
}
