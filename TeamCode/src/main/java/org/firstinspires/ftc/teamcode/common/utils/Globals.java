package org.firstinspires.ftc.teamcode.common.utils;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class Globals {
    public enum AllianceColor{
        BLUE,
        RED;

        public <T> T convert(Pose pose, Class<T> returnType) {
            if (this == BLUE) {
                return returnType.equals(Point.class)
                        ? returnType.cast(new Point(pose.getX(), pose.getY(), Point.CARTESIAN))
                        : returnType.cast(pose);
            }

            Pose transformedPose = transformPose(pose);
            return returnType.equals(Point.class)
                    ? returnType.cast(new Point(transformedPose.getX(), transformedPose.getY(), Point.CARTESIAN))
                    : returnType.cast(transformedPose);
        }

        public Point convert(Point point) {
            return this == BLUE ? point : transformPoint(point);
        }

        public double convertHeading(double heading) {
            return this == BLUE ? heading : Math.PI - heading;
        }

        // Private helper functions to remove redundant calculations
        private Pose transformPose(Pose pose) {
            return new Pose(144 - pose.getX(), 144 - pose.getY(), Math.PI - pose.getHeading());
        }

        private Point transformPoint(Point point) {
            return new Point(144 - point.getX(), 144 - point.getY(), Point.CARTESIAN);
        }

    }

    public static class Constants{

    }

    public static AllianceColor ALLIANCE = Globals.ALLIANCE.RED;
    public static boolean IS_AUTO = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean INTAKING_SAMPLES = false;
    public static boolean HOLDING_SAMPLE = false;
    public static boolean INTAKING_SPECIMENS = false;
    public static boolean HOLDING_SPECIMEN = false;

    public static final Pose sampleAutoStartPose = new Pose(6.595,101.105, Math.toRadians(270));
    public static final Pose preloadSampleStartPoseCorrected = new Pose(sampleAutoStartPose.getX()-6.595, sampleAutoStartPose.getY()+0.6, Math.toRadians(270));

    public static final Pose specAutoStartPose = new Pose(6.465,63.715, Math.toRadians(180));


    public static Pose END_OF_AUTO_POSE = new Pose(0, 0, Math.toRadians(0));
}
