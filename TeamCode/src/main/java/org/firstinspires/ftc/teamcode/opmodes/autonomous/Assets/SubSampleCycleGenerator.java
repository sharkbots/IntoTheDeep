package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;

import java.util.ArrayList;

public class SubSampleCycleGenerator {
    //private Pose subSampleLocation = new Pose(0, 0, 0);
    private ArrayList<Pose> subSampleLocations = new ArrayList<>();

    private Follower follower;

    public SubSampleCycleGenerator(){
    }

    public SubSampleCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }
    
    public PathChain getSubPickupPath() throws IllegalStateException{
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        return builder.build();
    }
}
