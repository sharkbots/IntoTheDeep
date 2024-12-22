package org.firstinspires.ftc.teamcode.common.utils;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;

public class Globals {
    public enum AllianceColor{
        BLUE,
        RED;

        public Pose convertPose(Pose pose) {
            if (this == BLUE)
                return pose;
            else
                return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
        }

        public Point convertPoint(Point point) {
            return this == BLUE ? point : new Point(144 - point.getX(), point.getY(), Point.CARTESIAN);
        }

//        public Pose getStartingPose() {
//            return this.convertPose(startingPose);
//        }
    }

    public static AllianceColor ALLIANCE = Globals.ALLIANCE.RED;
    public static boolean IS_AUTO = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean INTAKING = false;

    public static final Pose preloadSampleStartPose = new Pose(6.595-6.595,101.105, Math.toRadians(270));

    public static Pose END_OF_AUTO_POSE = new Pose();
}
