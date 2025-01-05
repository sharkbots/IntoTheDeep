package org.firstinspires.ftc.teamcode.common.utils;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.Point;

public class Globals {
    public enum AllianceColor{
        BLUE,
        RED;

        public Pose convertPose(Pose pose) {
            return this == BLUE ? pose : new Pose(144 - pose.getX(), 144 - pose.getY(), Math.PI - pose.getHeading());
        }

        public Point convertPoint(Point point) {
            return this == BLUE ? point : new Point(144 - point.getX(), 144 - point.getY(), Point.CARTESIAN);
        }

        public double convertHeading(double heading){
            return this == BLUE ? heading : Math.PI - heading;
        }

//        public Pose getStartingPose() {
//            return this.convertPose(startingPose);
//        }
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


    public static Pose END_OF_AUTO_POSE = new Pose();
}
