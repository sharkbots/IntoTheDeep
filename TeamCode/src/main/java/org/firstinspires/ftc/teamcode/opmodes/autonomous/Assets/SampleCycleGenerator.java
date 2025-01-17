package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class SampleCycleGenerator {
    public Pose bucketLocation = new Pose(12.386, 128.573, Math.toRadians(315));
    private Pose cycleBucketLocation = new Pose(bucketLocation.getX()+2, bucketLocation.getY()+2, Math.toRadians(315));
    private Pose outsideSampleLocation = new Pose(19.580, 126.069, Math.toRadians(20.11));
    private Pose middleSampleLocation = new Pose(19.001, 129.372, Math.toRadians(0));
    private Pose insideSampleLocation = new Pose(21.472, 128.140, Math.toRadians(339));


    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public SampleCycleGenerator(){

    }

    public SampleCycleGenerator(Pose bucketLocation, Pose outsideSampleLocation,
                                Pose middleSampleLocation, Pose insideSampleLocation) {
        this.bucketLocation = bucketLocation;
        this.insideSampleLocation = insideSampleLocation;
        this.middleSampleLocation = middleSampleLocation;
        this.outsideSampleLocation = outsideSampleLocation;
    }

    public SampleCycleGenerator setAlliance(Globals.AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public SampleCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public PathChain getSamplePath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(bucketLocation.getPoint()),
                    allianceColor.convertPoint(insideSampleLocation.getPoint())))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(339));

        else if (sampleLocation == SampleLocation.MIDDLE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(bucketLocation.getPoint()),
                    allianceColor.convertPoint(middleSampleLocation.getPoint())))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0));

        else if (sampleLocation == SampleLocation.OUTSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(bucketLocation.getPoint()),
                    allianceColor.convertPoint(middleSampleLocation.getPoint())))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(22.11));

        return builder.setZeroPowerAccelerationMultiplier(1).build();
    }

    public PathChain getBucketPath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(insideSampleLocation.getPoint()),
                            allianceColor.convertPoint(new Point(insideSampleLocation.getX()+2, insideSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(allianceColor.convertHeading(insideSampleLocation.getHeading()), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(new Point(insideSampleLocation.getX()+2, insideSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convertPoint(cycleBucketLocation.getPoint())))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        else if (sampleLocation == SampleLocation.MIDDLE) {
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(middleSampleLocation.getPoint()),
                            allianceColor.convertPoint(new Point(middleSampleLocation.getX()+2, middleSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(Math.toRadians(allianceColor.convertHeading(middleSampleLocation.getHeading())), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(new Point(middleSampleLocation.getX()+2, middleSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convertPoint(cycleBucketLocation.getPoint())))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        else if (sampleLocation == SampleLocation.OUTSIDE) {
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(outsideSampleLocation.getPoint()),
                            allianceColor.convertPoint(new Point(outsideSampleLocation.getX(), outsideSampleLocation.getY()-3, Point.CARTESIAN))))
                    .setLinearHeadingInterpolation(Math.toRadians(allianceColor.convertHeading(outsideSampleLocation.getHeading())), allianceColor.convertHeading(cycleBucketLocation.getHeading()));
            builder.addPath(new BezierLine(
                            allianceColor.convertPoint(new Point(outsideSampleLocation.getX(), outsideSampleLocation.getY()-3, Point.CARTESIAN)),
                            allianceColor.convertPoint(cycleBucketLocation.getPoint())))
                    .setConstantHeadingInterpolation(allianceColor.convertHeading(cycleBucketLocation.getHeading()));
        }

        return builder
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setPathEndTValueConstraint(0.9)
                .build();
    }

    public enum SampleLocation {
        OUTSIDE, MIDDLE, INSIDE
    }
}
