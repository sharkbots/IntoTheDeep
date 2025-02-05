package org.firstinspires.ftc.teamcode.opmodes.autonomous.Assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.utils.Globals;

public class SpecimenCycleGenerator {
    public static Pose pickupLocation = new Pose(7.595, 32.342, Math.toRadians(180));
    public static Pose intermediatePickupLocation = new Pose(pickupLocation.getX()+10, pickupLocation.getY(), pickupLocation.getHeading());
    public static Pose depositLocation = new Pose(38, 65.72/*63.72*/, Math.toRadians(180));
    public static Pose intermediateDepositLocation = new Pose(depositLocation.getX(), depositLocation.getY()-3, depositLocation.getHeading());
    public static Pose depositSetupLocation = new Pose(depositLocation.getX()-10, intermediateDepositLocation.getY(), depositLocation.getHeading());
    private double depositGap = 1.5;

    private Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
    private Follower follower;

    public SpecimenCycleGenerator(){

    }

    public SpecimenCycleGenerator setAlliance(Globals.AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public SpecimenCycleGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public PathChain getDepositPath(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                new BezierCurve(
                        allianceColor.convert(pickupLocation, Point.class),
                        allianceColor.convert(new Point(20.738, 61.437, Point.CARTESIAN)),
                        allianceColor.convert(depositSetupLocation, Point.class)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));

        builder.addPath(
                        new BezierLine(
                                allianceColor.convert(depositSetupLocation, Point.class),
                                allianceColor.convert(
                                        new Point(depositLocation.getX(), depositLocation.getY()-cycleNum*depositGap))))
                .setPathEndVelocityConstraint(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(500);

        return builder
                .build();
    }


    public PathChain getPickupPath(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                new BezierCurve(
                        allianceColor.convert(
                                new Point(depositLocation.getX(), depositLocation.getY()-(cycleNum-1)*depositGap)
                        ),
                        new Point(17.830, 58.724, Point.CARTESIAN),
                        new Point(31.397, 27.715, Point.CARTESIAN),
                        allianceColor.convert(intermediatePickupLocation, Point.class)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        builder.addPath(
                        new BezierLine(
                                allianceColor.convert(intermediatePickupLocation, Point.class),
                                allianceColor.convert(pickupLocation, Point.class)))
                .setPathEndVelocityConstraint(2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.99);

        return builder
                .build();
    }
}
