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

    public static AllianceColor ALLIANCE = AllianceColor.RED;
    public static boolean IS_AUTO = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean INTAKING_SAMPLES = false;
    public static boolean HOLDING_SAMPLE = false;
    public static boolean INTAKING_SPECIMENS = false;
    public static boolean HOLDING_SPECIMEN = false;

    // Size of robot (in inches)
    public static double ROBOT_LENGTH = 13.1;
    public static double ROBOT_WIDTH = 12.8;

    // Pivot times
    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double INTAKE_ARM_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    public static double INTAKE_CLAW_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    public static double DEPOSIT_CLAW_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer

    // Intake Arm Pivot
    public static double INTAKE_ARM_PIVOT_TRANSFER_POS = 0.47;
    public static double INTAKE_ARM_PIVOT_HOVER_INTAKE_POS = 0.75;
    public static double INTAKE_ARM_PIVOT_INTAKE_POS = 0.79;
    public static double INTAKE_ARM_PIVOT_HOVER_WITH_SAMPLE_POS = 0.7;

    // Intake Claw Pivot
    public static double INTAKE_CLAW_PIVOT_TRANSFER_POS = 0.24;
    public static double INTAKE_CLAW_PIVOT_HOVER_INTAKE_POS = 0.95;
    public static double INTAKE_CLAW_PIVOT_INTAKE_POS = 0.9;
    public static double INTAKE_CLAW_PIVOT_HOLDING_POS = 1;

    // Intake Claw Rotation
    public static double INTAKE_CLAW_ROTATION_TRANSFER_POS = 0.54;

    // Intake Claw
    public static double INTAKE_CLAW_OPEN_POS = 0.73;
    public static double INTAKE_CLAW_MICRO_OPEN_POS = 0.95;
    public static double INTAKE_CLAW_CLOSED_POS = 0.98;

    // Intake Extendo
    public static int MAX_EXTENDO_EXTENSION = 1850;

    // Deposit Claw Pivot
    public static double DEPOSIT_CLAW_PIVOT_TRANSFER_POS = 0.045;
    public static double DEPOSIT_CLAW_PIVOT_BUCKET_POS = 0.81;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_INTAKE_POS = 0.96;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_POS = 0.88;

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.71;
    public static double DEPOSIT_CLAW_MICRO_OPEN_POS = 0.0;
    public static double DEPOSIT_CLAW_CLOSED_POS = 0.96;

    // Deposit Claw Rotation
    public static double DEPOSIT_CLAW_ROTATION_TRANSFER_POS = 0.35;
    public static double DEPOSIT_CLAW_ROTATION_BUCKET_SCORING_RED_POS = 0.02;
    public static double DEPOSIT_CLAW_ROTATION_BUCKET_SCORING_BLUE_POS = 0.685;

    // Deposit Slides
    public static int MAX_SLIDES_EXTENSION = 1900;
    public static int HOLDING_SPECIMEN_HEIGHT = 50;
    public static int LOW_BUCKET_HEIGHT = 925;
    public static int HIGH_BUCKET_HEIGHT = 1875;
    public static int LOW_SPECIMEN_HEIGHT = 85;
    public static int HIGH_SPECIMEN_HEIGHT = 540;
    public static int LVL1_HANG_HEIGHT = 725;
    public static int ENDGAME_ASCENT_HEIGHT = 0;

    // Autonomous poses
    public static final Pose sampleAutoStartPose = new Pose(6.595, 101.105, Math.toRadians(270));
    public static final Pose preloadSampleStartPoseCorrected = new Pose(sampleAutoStartPose.getX() - 6.595, sampleAutoStartPose.getY() + 0.6, Math.toRadians(270));
    public static final Pose specAutoStartPose = new Pose(6.465, 63.715, Math.toRadians(180));

    public static Pose END_OF_AUTO_POSE = new Pose(0, 0, Math.toRadians(0));
}