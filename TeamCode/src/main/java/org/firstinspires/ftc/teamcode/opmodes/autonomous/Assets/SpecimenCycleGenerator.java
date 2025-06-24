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
    public static Pose pickupLocation = new Pose(9.595, 32.342, Math.toRadians(0));
    public static Pose intermediatePickupLocation = new Pose(pickupLocation.getX()+5, pickupLocation.getY(), pickupLocation.getHeading());
    public static Pose depositLocation = new Pose(40-1.5, 67.72/*+1.5+2*/, Math.toRadians(0));
    //public static Pose intermediateDepositLocation = new Pose(depositLocation.getX(), depositLocation.getY()-3, depositLocation.getHeading());
    //public static Pose depositSetupLocation = new Pose(depositLocation.getX()-10, intermediateDepositLocation.getY(), depositLocation.getHeading());
    private final double depositGap = 1.5;
    private final double constantCycleShift = 1;

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

    public PathChain getDepositPathSpline(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                        new BezierCurve(
                                allianceColor.convert(pickupLocation, Point.class),
                                new Point(depositLocation.getX()-5, depositLocation.getY()-constantCycleShift-cycleNum*depositGap),
                                new Point(depositLocation.getX()-5, depositLocation.getY()-constantCycleShift-cycleNum*depositGap),
                                allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-cycleNum*depositGap))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(4);

        return builder
                .build();
    }

    public PathChain getPickupPathSpline(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                        new BezierCurve(
                                allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-(cycleNum-1)*depositGap)),
                                new Point(30.44, depositLocation.getY()-constantCycleShift-(cycleNum-1)*depositGap),
                                new Point(45.3, pickupLocation.getY()),
                                allianceColor.convert(pickupLocation, Point.class)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.97)
                //.addParametricCallback(0.7, ()-> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(4.25);

        return builder
                .build();
    }



    public PathChain getDepositPathStrafe(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        if (cycleNum == 1){
            builder.addPath(
                            new BezierLine(
                                    allianceColor.convert(pickupLocation, Point.class),
                                    allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-1.5-cycleNum*depositGap))))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTValueConstraint(0.95)
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndHeadingConstraint(Math.toRadians(3));
        }
        else if (cycleNum == 5){
            builder.addPath(
                            new BezierCurve(
                                    allianceColor.convert(pickupLocation, Point.class),
                                    new Point(depositLocation.getX()-5, depositLocation.getY()-constantCycleShift-cycleNum*depositGap),
                                    new Point(depositLocation.getX()-5, depositLocation.getY()-constantCycleShift-cycleNum*depositGap),
                                    allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-cycleNum*depositGap))))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTValueConstraint(0.95)
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndHeadingConstraint(Math.toRadians(3));
        }
        else builder.addPath(
                        new BezierLine(
                                allianceColor.convert(pickupLocation, Point.class),
                                allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-cycleNum*depositGap))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.95)
                .setZeroPowerAccelerationMultiplier(4);

//        builder.addPath(
//                new BezierCurve(
//                        allianceColor.convert(pickupLocation, Point.class),
//                        allianceColor.convert(new Point(20.738, 61.437, Point.CARTESIAN)),
//                        allianceColor.convert(new Point(depositLocation.getX()-10, depositLocation.getY()-cycleNum*depositGap))))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));
//
//        builder.addPath(
//                        new BezierLine(
//                                allianceColor.convert(new Point(depositLocation.getX()-10, depositLocation.getY()-cycleNum*depositGap)),
//                                allianceColor.convert(
//                                        new Point(depositLocation.getX(), depositLocation.getY()-cycleNum*depositGap))))
//                .setPathEndVelocityConstraint(3)
//                .setPathEndTValueConstraint(0.99)
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setPathEndTimeoutConstraint(250)
//                .addParametricCallback(0.05, ()-> follower.setMaxPower(1.0));

        return builder
                .build();
    }

    public PathChain getPickupPathStrafe(int cycleNum) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = follower.pathBuilder();

        builder.addPath(
                new BezierLine(
                        allianceColor.convert(new Point(depositLocation.getX(), depositLocation.getY()-constantCycleShift-(cycleNum-1)*depositGap)),
                        allianceColor.convert(intermediatePickupLocation, Point.class)))
                .setZeroPowerAccelerationMultiplier(6)
                .setConstantHeadingInterpolation(Math.toRadians(0));

        builder.addPath(
                new BezierLine(
                        allianceColor.convert(intermediatePickupLocation, Point.class),
                        allianceColor.convert(pickupLocation, Point.class)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.99);


//
//        builder.addPath(
//                        new BezierLine(
//                                allianceColor.convert(intermediatePickupLocation, Point.class),
//                                allianceColor.convert(pickupLocation, Point.class)))
//                .setPathEndVelocityConstraint(3)
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addParametricCallback(0.1, ()-> follower.setMaxPower(0.7))
//                .setPathEndTValueConstraint(0.99)
//                .setPathEndTimeoutConstraint(200);
        return builder
                .build();
    }
}