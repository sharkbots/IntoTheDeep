package org.firstinspires.ftc.teamcode.common.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

@Config
public class Globals {
    public static class SampleAutonomousConfig{
        double samp1X = 48;
        double samp1Y = 60;
        double samp2X = 48;
        double samp2Y = 48;
    }

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

    // Camera configurations
    public static long CAMERA_EXPOSURE_MILLIS = 25; //33
    public static int CAMERA_WHITE_BALANCE_TEMPERATURE = 5500; //5500
    public static int CAMERA_GAIN = 30; //70
    public static int CAMERA_GAUSSIAN = 11; //70

    public static int CAMERA_STREAM_WIDTH = 800;
    public static int CAMERA_STREAM_HEIGHT = 600;

    public final static double CAMERA_OFFSET_FROM_CENTER_Y_IN = 3.0/8.0;


    public static double DEFAULT_LIFT_FEEDFORWARD = 0.08;
    public static double LIFT_RESET_FEEDFORWARD = -0.0;
    public static double LIFT_NEAR_RESET_FEEDFORWARD = 0.0;
    public static double EXTENDO_FEEDFORWARD_EXTENDING = 0.2;
    public static double EXTENDO_FEEDFORWARD_RETRACTING = -0.2;

    public static AllianceColor ALLIANCE = AllianceColor.RED;
    public static boolean IS_AUTONOMOUS = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean INTAKING_SAMPLES = false;
    public static boolean HOLDING_SAMPLE = false;
    public static boolean INTAKING_SPECIMENS = false;
    public static boolean HOLDING_SPECIMEN = false;

    public static boolean IS_DT_MANUAL_CONTROL = false;
    public static boolean IS_DT_AUTO_ALIGNING = false;
    public static double HOLDPOINT_MANUAL_FEEDFORWARD = 0.5;

    // Size of robot (in inches)
    public static double ROBOT_LENGTH = 13.1;
    public static double ROBOT_WIDTH = 12.8;

    // Pivot time (axon max no SPM)
    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double INTAKE_ARM_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    public static double INTAKE_CLAW_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    public static double DEPOSIT_CLAW_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer

    // Rotation time (GB speed no SPM)
    // 0.xx sec/180° -> 0.xxx sec/300° -> xxx milliseconds/300°
    public static double INTAKE_CLAW_ROTATION_MOVEMENT_TIME = 200;

    public static double AA_claw_rotation_heading_degrees = 0;
    // Intake Arm Pivot
    public static double INTAKE_ARM_PIVOT_TRANSFER_POS = 0.47;
    public static double INTAKE_ARM_PIVOT_HOVER_WITH_SAMPLE_POS = 0.7;
    public static double INTAKE_ARM_PIVOT_HOVER_INTAKE_POS = 0.65;
    public static double INTAKE_ARM_PIVOT_INTAKE_POS = 0.79; // 0.79

    // Intake Claw Pivot
    public static double INTAKE_CLAW_PIVOT_TRANSFER_POS = 0.015;
    public static double INTAKE_CLAW_PIVOT_HOVER_INTAKE_POS = 0.735; //0.79
    public static double INTAKE_CLAW_PIVOT_INTAKE_POS = 0.675;
    public static double INTAKE_CLAW_PIVOT_HOLDING_POS = 0.775;

    // Intake Claw Rotation
    public static double INTAKE_CLAW_ROTATION_TRANSFER_POS = 0.54;
    public static double INTAKE_CLAW_ROTATION_FULLY_LEFT_POS = 0.88;
    public static double INTAKE_CLAW_ROTATION_FULLY_RIGHT_POS = 0.20;


    // Intake Claw
    public static double INTAKE_CLAW_OPEN_POS = 0.48;
    public static double INTAKE_CLAW_MICRO_OPEN_POS = 0.765;
    public static double INTAKE_CLAW_CLOSED_POS = 0.785;

    // Extendo
    public static int MAX_EXTENDO_EXTENSION = 1850;
    public static int EXTENDO_FEEDFORWARD_TRIGGER_THRESHOLD = 1300;

    // 2143 ticks / 21.26 inch --> 100.7996 ticks / 1 inch
    public static double EXTENDO_TICKS_PER_INCH = 100.8;
    // xxx sec / xxx ticks --> xxx sec / xxx ticks
    public static double EXTENDO_MOVEMENT_TIME = 0;

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
    public static int MAX_SLIDES_EXTENSION = 2050;
    public static int HOLDING_SPECIMEN_HEIGHT = 65;
    public static int LOW_BUCKET_HEIGHT = 1000;
    public static int HIGH_BUCKET_HEIGHT = 1970;
    public static int LOW_SPECIMEN_HEIGHT = 85;
    public static int HIGH_SPECIMEN_SETUP_HEIGHT = 925;
    public static int HIGH_SPECIMEN_HEIGHT = 600;
    public static int LVL1_ASCENT_HEIGHT = 825;
    public static int ENDGAME_ASCENT_SETUP_HEIGHT = 2000;
    public static int ENDGAME_ASCENT_HEIGHT = 1100;
    public static int POST_BUZZER_HANG_RELEASE_HEIGHT = 1550; // 1600 before, which tilted the robot


    // Autonomous poses

    // Sample autonomous poses
    public static final Pose sampleAutoStartPose = new Pose(6.595, 111.355, Math.toRadians(270));
    public static final Pose preloadSampleStartPoseCorrected = new Pose(sampleAutoStartPose.getX() - 6.595, sampleAutoStartPose.getY() + 0.6, Math.toRadians(270));
    public static final Pose bucketPose = new Pose(12.386, 128.573, Math.toRadians(315));
    // Specimen auto pose
    public static final Pose specAutoStartPose = new Pose(6.465, 63.715, Math.toRadians(180));

    public static Pose END_OF_AUTO_POSE = new Pose(36, 36, Math.toRadians(90));
    //public static Pose END_OF_AUTO_POSE = new Pose(0, 0, Math.toRadians(0));

}