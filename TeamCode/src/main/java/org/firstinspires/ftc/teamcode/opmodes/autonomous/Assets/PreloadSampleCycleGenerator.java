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
    private Pose outsideSampleLocation = new Pose(25, 130, Math.toRadians(25.6));
    private Pose middleSampleLocation = new Pose(25, 130, Math.toRadians(0));
    private Pose insideSampleLocation = new Pose(25, 130, Math.toRadians(333.8));


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
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), insideSampleLocation.getHeading());

        else if (sampleLocation == SampleLocation.MIDDLE)
            builder.addPath(new BezierLine(
                    allianceColor.convert(bucketLocation, Point.class),
                    allianceColor.convert(middleSampleLocation, Point.class)))
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), middleSampleLocation.getHeading());

        else if (sampleLocation == SampleLocation.OUTSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convert(bucketLocation, Point.class),
                    allianceColor.convert(middleSampleLocation, Point.class)))
                    .setLinearHeadingInterpolation(bucketLocation.getHeading(), outsideSampleLocation.getHeading());

        return builder.setZeroPowerAccelerationMultiplier(1).build();
    }

    public PathChain getBucketPath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(insideSampleLocation, Point.class),
                            allianceColor.convert(new Point(insideSampleLocation.getX()+2, insideSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(allianceColor.convertHeading(insideSampleLocation.getHeading()), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convert(new Point(insideSampleLocation.getX()+2, insideSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convert(cycleBucketLocation, Point.class)))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        else if (sampleLocation == SampleLocation.MIDDLE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(middleSampleLocation, Point.class),
                            allianceColor.convert(new Point(middleSampleLocation.getX()+2, middleSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(Math.toRadians(allianceColor.convertHeading(middleSampleLocation.getHeading())), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convert(new Point(middleSampleLocation.getX()+2, middleSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convert(cycleBucketLocation, Point.class)))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        else if (sampleLocation == SampleLocation.OUTSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convert(outsideSampleLocation, Point.class),
                            allianceColor.convert(new Point(outsideSampleLocation.getX(), outsideSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(Math.toRadians(allianceColor.convertHeading(outsideSampleLocation.getHeading())), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convert(new Point(outsideSampleLocation.getX(), outsideSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convert(cycleBucketLocation, Point.class)))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        return builder
                .addParametricCallback(0.5, () -> follower.setMaxPower(1))
                .setPathEndTValueConstraint(0.9)
                .build();
    }

    public enum SampleLocation {
        OUTSIDE, MIDDLE, INSIDE
    }
}
