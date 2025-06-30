package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class PreloadSampleCycleGenerator {
    public Pose bucketLocation = Globals.bucketPose;
    private Pose cycleBucketLocation = new Pose(bucketLocation.getX()+2, bucketLocation.getY()+2, Math.toRadians(315));
    private Pose insideSampleLocation = new Pose(24, 119, Math.toRadians(0));
    private Pose middleSampleLocation = new Pose(24, 129+0.5, Math.toRadians(0));
    private Pose outsideSampleLocation = new Pose(28.74, 124.3, Math.toRadians(51.77-2-3-2-2));


    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public PreloadSampleCycleGenerator(){

    }

    public PreloadSampleCycleGenerator(Pose bucketLocation, Pose outsideSampleLocation,
                                       Pose middleSampleLocation, Pose insideSampleLocation) {
        this.bucketLocation = bucketLocation;
        this.insideSampleLocation = insideSampleLocation;
        this.middleSampleLocation = middleSampleLocation;
        this.outsideSampleLocation = outsideSampleLocation;
    }

    public PreloadSampleCycleGenerator setAlliance(Globals.AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public PreloadSampleCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public PathChain getSamplePath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convert(bucketLocation, Point.class),
                    allianceColor.convert(insideSampleLocation, Point.class)))
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), insideSampleLocation.getHeading())
                    .setZeroPowerAccelerationMultiplier(2)
                    .addParametricCallback(0.3, () -> follower.setMaxPower(0.4));

        else if (sampleLocation == SampleLocation.MIDDLE)
            builder.addPath(new BezierLine(
                    allianceColor.convert(bucketLocation, Point.class),
                    allianceColor.convert(middleSampleLocation, Point.class)))
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), middleSampleLocation.getHeading())
                    .setZeroPowerAccelerationMultiplier(2)
                    .addParametricCallback(0.6, () -> follower.setMaxPower(0.6));

        else if (sampleLocation == SampleLocation.OUTSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convert(bucketLocation, Point.class),
                    allianceColor.convert(outsideSampleLocation, Point.class)))
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), outsideSampleLocation.getHeading())
                    .setZeroPowerAccelerationMultiplier(2)
                    .addParametricCallback(0.6, () -> follower.setMaxPower(0.6));

        return builder.build();
    }

    public PathChain getBucketPath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(insideSampleLocation, Point.class),
                            allianceColor.convert(bucketLocation, Point.class)))
                    .setLinearHeadingInterpolation(insideSampleLocation.getHeading(), bucketLocation.getHeading());
        }

        else if (sampleLocation == SampleLocation.MIDDLE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(middleSampleLocation, Point.class),
                            allianceColor.convert(bucketLocation, Point.class)))
                    .setLinearHeadingInterpolation(middleSampleLocation.getHeading(), bucketLocation.getHeading());
        }

        else if (sampleLocation == SampleLocation.OUTSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(outsideSampleLocation, Point.class),
                            allianceColor.convert(bucketLocation, Point.class)))
                    .setLinearHeadingInterpolation(outsideSampleLocation.getHeading(), bucketLocation.getHeading());
        }

        return builder
                .addParametricCallback(0.1, () -> follower.setMaxPower(1.0))
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.98)
                .build();
    }

    public enum SampleLocation {
        OUTSIDE, MIDDLE, INSIDE
    }
}
