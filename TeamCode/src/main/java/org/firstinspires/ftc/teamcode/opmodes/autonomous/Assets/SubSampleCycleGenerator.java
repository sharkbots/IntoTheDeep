package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.math.MathUtils;

import java.util.ArrayList;

public class SubSampleCycleGenerator {
    //private Pose subSampleLocation = new Pose(0, 0, 0);
    private ArrayList<Pose> subSampleLocations = new ArrayList<>();
    public Pose bucketLocation = Globals.bucketPose;

    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public SubSampleCycleGenerator(){
    }

    public SubSampleCycleGenerator setAlliance(Globals.AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public SubSampleCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public void addSubSampleLocation(Pose subSampleLocation, int cycleNum){
        if (subSampleLocations.size()<cycleNum){
            subSampleLocations.add(subSampleLocation);
        }
        else subSampleLocations.set(cycleNum-1, subSampleLocation);
    }

    public PathChain getSubPickupPath(int cycleNum) throws IllegalStateException{
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(new BezierCurve(
                        allianceColor.convert(bucketLocation, Point.class),
                        new Point(subSampleLocations.get(cycleNum-1).getX(), 117.5),
                        new Point(subSampleLocations.get(cycleNum-1).getX(), 92)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
                .setPathEndTValueConstraint(0.94)
                //.setPathEndTimeoutConstraint(0.97)
                .setZeroPowerAccelerationMultiplier(2);
        return builder.build();
    }

    public PathChain getSubDepositPath(int cycleNum) throws IllegalStateException{
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(new BezierCurve(
                new Point(subSampleLocations.get(cycleNum-1).getX(), 92),
                new Point(subSampleLocations.get(cycleNum-1).getX(), 117.5),
                allianceColor.convert(bucketLocation, Point.class)
        )).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                .setPathEndTValueConstraint(0.98);

        return builder.build();

    }

//    public PathChain getSubPickupPath(int cycleNum) throws IllegalStateException{
//        if (follower == null)
//            throw new IllegalStateException("The generator's follower wasn't set");
//
//        PathBuilder builder = follower.pathBuilder();
//
//        builder.addPath(new BezierCurve(
//                allianceColor.convert(bucketLocation, Point.class),
//                new Point(subSampleLocations.get(cycleNum-1))
//        ))
//                .setTangentHeadingInterpolation()
//                .setPathEndTimeoutConstraint(0.95);
//        return builder.build();
//    }
//
//    public PathChain getSubDepositPath(int cycleNum) throws IllegalStateException{
//        if (follower == null)
//            throw new IllegalStateException("The generator's follower wasn't set");
//
//        PathBuilder builder = follower.pathBuilder();
//
//        builder.addPath(new BezierCurve(
//                new Point(subSampleLocations.get(cycleNum-1)),
//                allianceColor.convert(bucketLocation, Point.class)
//        )).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
//                .addTemporalCallback(0.8, ()-> follower.setMaxPower(0.7));
//
//        return builder.build();
//
//    }
}
