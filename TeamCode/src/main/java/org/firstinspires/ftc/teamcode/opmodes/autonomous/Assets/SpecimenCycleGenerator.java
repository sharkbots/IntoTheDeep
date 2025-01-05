package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class SpecimenCycleGenerator {
    public Pose pickupLocation = new Pose(7.595, 36.342, Math.toRadians(180));
    private Pose DepositLocation = new Pose(38.913, 63.72, Math.toRadians(180));
    private Pose intermediateDepositLocation = new Pose(DepositLocation.getX(), DepositLocation.getY()-3, DepositLocation.getHeading());


    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public SpecimenCycleGenerator(){

    }

    public SpecimenCycleGenerator(Pose pickupLocation, Pose DepositLocation,
                                  Pose intermediateDepositLocation) {
        this.pickupLocation = pickupLocation;
        this.DepositLocation = DepositLocation;
        this.intermediateDepositLocation = intermediateDepositLocation;
    }

    public SpecimenCycleGenerator setAlliance(Globals.AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public SpecimenCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public PathChain getDepositPath() throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                new BezierCurve(
                        allianceColor.convertPoint(pickupLocation.getPoint()),
                        allianceColor.convertPoint(new Point(20.738, 61.437, Point.CARTESIAN)),
                        allianceColor.convertPoint(DepositLocation.getPoint())))
                .setPathEndVelocityConstraint(3)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        return builder
                .build();
    }

    public PathChain getPickupPath() throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                new BezierCurve(
                        allianceColor.convertPoint(DepositLocation.getPoint()),
                        new Point(17.830, 58.724, Point.CARTESIAN),
                        new Point(31.397, 27.715, Point.CARTESIAN),
                        allianceColor.convertPoint(pickupLocation.getPoint())))
                .setPathEndVelocityConstraint(3)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));


        return builder
                .build();
    }

    public enum SampleLocation {
        OUTSIDE, MIDDLE, INSIDE
    }
}
