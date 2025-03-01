package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.utils.Globals;

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

    public SubSampleCycleGenerator addSubSampleLocation(Pose subSampleLocation){
        subSampleLocations.add(subSampleLocation);
        return this;
    }
    
    public PathChain getSubPickupPath(int cycleNum) throws IllegalStateException{
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();
        builder.addPath(new BezierLine(
                allianceColor.convert(bucketLocation, Point.class),
                new Point(60, 103)
        )).setTangentHeadingInterpolation();

        builder.addPath(new BezierLine(
                new Point(60, 103),
                new Point(60, 104)
        )).setConstantHeadingInterpolation(Math.toRadians(270));

        builder.addPath(new BezierLine(
                new Point(60, 104),
                new Point(subSampleLocations.get(cycleNum).getX(),90.5)
        )).setConstantHeadingInterpolation(Math.toRadians(270));

        return builder.build();
    }
}
