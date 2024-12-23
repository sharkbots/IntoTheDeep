package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class SampleCycleGenerator {
    private Point bucketLocation = new Point(12.386, 128.573, Point.CARTESIAN);
    private Point outsideSampleLocation = new Point(19.580, 126.069, Point.CARTESIAN);
    private Point middleSampleLocation = new Point(19.001, 129.372, Point.CARTESIAN);
    private Point insideSampleLocation = new Point(21.472, 128.140, Point.CARTESIAN);


    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public SampleCycleGenerator(){

    }

    public SampleCycleGenerator(Point bucketLocation, Point outsideSampleLocation,
                                Point middleSampleLocation, Point insideSampleLocation) {
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
                    allianceColor.convertPoint(bucketLocation),
                    allianceColor.convertPoint(insideSampleLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(339));

        else if (sampleLocation == SampleLocation.MIDDLE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(bucketLocation),
                    allianceColor.convertPoint(middleSampleLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0));

        else if (sampleLocation == SampleLocation.OUTSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(bucketLocation),
                    allianceColor.convertPoint(middleSampleLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(23.11));

        return builder.setZeroPowerAccelerationMultiplier(1).build();
    }

    public PathChain getBucketPath(SampleLocation sampleLocation) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (sampleLocation == SampleLocation.INSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(insideSampleLocation),
                    allianceColor.convertPoint(bucketLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(345), Math.toRadians(315));

        else if (sampleLocation == SampleLocation.MIDDLE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(middleSampleLocation),
                    allianceColor.convertPoint(bucketLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315));

        else if (sampleLocation == SampleLocation.OUTSIDE)
            builder.addPath(new BezierLine(
                    allianceColor.convertPoint(outsideSampleLocation),
                    allianceColor.convertPoint(bucketLocation)))
                    .setLinearHeadingInterpolation(Math.toRadians(23.11), Math.toRadians(315));

//        builder.addParametricCallback(0.1, () -> follower.setMaxPower(1.0))
//                .setConstantHeadingInterpolation(Math.toRadians(-90));

        return builder.build();
    }

    public enum SampleLocation {
        OUTSIDE, MIDDLE, INSIDE
    }
}
