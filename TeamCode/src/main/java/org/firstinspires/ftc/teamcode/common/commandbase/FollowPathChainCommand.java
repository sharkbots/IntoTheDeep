package org.firstinspires.ftc.teamcode.common.commandbase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class FollowPathChainCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private double completionThreshold = FollowerConstants.pathEndTValueConstraint;
    private boolean useIsBusy = true;
    private final Robot robot;

    public FollowPathChainCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
        robot = Robot.getInstance();
    }

    public FollowPathChainCommand(Follower follower, Path path) {
        this(follower, new PathChain(path));
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowPathChainCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    public FollowPathChainCommand setCompletionThreshold(double threshold) {
        this.completionThreshold = threshold;
        return this;
    }

    public FollowPathChainCommand disableUseIsBusy(){
        this.useIsBusy = false;
        return this;
    }

    public FollowPathChainCommand enableUseIsBusy(){
        this.useIsBusy = true;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        if (useIsBusy) {
            return !follower.isBusy();

        } else {
            robot.telemetryA.addData("Current Path Number (not done)", follower.getCurrentPathNumber());
            robot.telemetryA.update();
            if (follower.getCurrentPathNumber() == this.path.size() - 1 /*&& Math.abs(follower. headingError) < 0.1*/) {
                robot.telemetryA.addData("Current Path Number (done)", follower.getCurrentPathNumber());
                robot.telemetryA.addData("Current completion threshold (done)", this.completionThreshold);
                robot.telemetryA.addData("t value (done)", follower.getCurrentTValue());
                robot.telemetryA.update();
                return follower.getCurrentTValue() >=
                        this.completionThreshold;
            }
            return false;
        }

//    @Override
//    public boolean isFinished(){
//        if (follower.getCurrentPathNumber () == this.path.size() - 1 /*&& Math.abs(follower. headingError) < 0.1*/) {
//            return follower.getCurrentTValue() >=
//            this.completionThreshold;
//        }
//        return false;
//    }
    }
}