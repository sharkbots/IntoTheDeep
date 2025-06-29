package org.firstinspires.ftc.teamcode.common.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.vision.Sample;

@Config
public class Globals {

    // ** Autonomous poses **

    // Sample autonomous poses
    public static final Pose sampleAutoStartPose = new Pose(6.595, 111.355, Math.toRadians(270));
    public static final Pose bucketPose = new Pose(12.786, 129.573, Math.toRadians(315));
    // Specimen auto pose
    public static final Pose specAutoStartPose = new Pose(6.465, 63.715, Math.toRadians(0));

    public static Pose END_OF_AUTO_POSE = new Pose(36, 36, Math.toRadians(90));

    public static double seamsToInches(double seams){
        if (seams == 0) return 0.0;
        else if (Math.abs(seams) <= 0.5) return Math.signum(seams)*3.0/8;
        else if (Math.abs(seams) <= 1.0) return Math.signum(seams)*6.0/8;
        else return 1.875 + 18.0/16 * (seams-1);
    }


    public static class SpecAutonomousConfig  {
        public static Globals.AllianceColor allianceColor = Globals.AllianceColor.RED;
        public static Globals.GRABBING_MODES grabbingMode = GRABBING_MODES.SPECIMEN;

        public static double samp1X = 0;
        public static double samp1Y = 12;
        public static double samp1Angle = 0;

        public static double samp2X = 0;
        public static double samp2Y = 12;
        public static double samp2Angle = 0;

    }

    public static class SampleAutonomousConfig  {
        public static Globals.AllianceColor allianceColor = Globals.AllianceColor.BLUE;
        public static Globals.GRABBING_MODES grabbingMode = GRABBING_MODES.SAMPLE;
        public static boolean grabSecondPreload = false;
        //        public static int numSubCycles = 1;
        public static double samp1X = -4;
        public static double samp1Y = 12;
        public static double samp2X = -4;
        public static double samp2Y = 12;

        public static long waitOZinSeconds = 0;
//        public static double samp2X = 0;
//        public static double samp2Y = 0;

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

    public enum GRABBING_MODES {
        SAMPLE,
        SPECIMEN;

        private static GRABBING_MODES current = SAMPLE;

        public static GRABBING_MODES current() {
            return current;
        }

        public static void set(GRABBING_MODES mode) {
            current = mode;
        }

        public static void next() {
            switch (current) {
                case SAMPLE:
                    current = SPECIMEN;
                    break;
                case SPECIMEN:
                    current = SAMPLE;
                    break;
            }
        }

        public static int[] getControllerColor() {
            switch (current) {
                case SAMPLE:
                    return new int[]{255, 255, 0};
                case SPECIMEN:
                    return (Globals.ALLIANCE_COLOR == Globals.AllianceColor.BLUE)
                            ? new int[]{0, 0, 255}
                            : new int[]{255, 0, 0};
                default:
                    return new int[]{0, 255, 0};
            }
        }
    }


    // Camera configurations
    public static boolean FREEZE_CAMERA_FRAME = false;
    public static String COLOR_SAMPLE_FILTERING = "yellow";

    public static long CAMERA_EXPOSURE_MILLIS = 25; //33
    public static int CAMERA_WHITE_BALANCE_TEMPERATURE = 5500; //5500
    public static int CAMERA_GAIN = 30; //70
    public static int CAMERA_GAUSSIAN = 11; //70

    public static int CAMERA_STREAM_WIDTH = 800;
    public static int CAMERA_STREAM_HEIGHT = 600;

    public final static double CAMERA_OFFSET_FROM_CENTER_Y_IN = 3.0/8.0;

    public static double DEFAULT_LIFT_FEEDFORWARD = 0.08;
    public static double LIFT_RESET_FEEDFORWARD = -0.2;
    public static double LIFT_NEAR_RESET_FEEDFORWARD = 0.2;
    public static double EXTENDO_FEEDFORWARD_EXTENDING = 0.0;
    public static double EXTENDO_FEEDFORWARD_RETRACTING = -0.0;

    public static AllianceColor ALLIANCE_COLOR;
    public static Sample.Color GRABBING_COLOR;
    public static boolean IS_AUTONOMOUS = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = false;
    public static boolean INTAKING_SAMPLES = false;
    public static boolean HOLDING_SAMPLE = false;
    public static boolean INTAKING_SPECIMENS = false;
    public static boolean HOLDING_SPECIMEN = false;
    public static boolean INTAKE_JUST_CANCELLED = false;
    public static boolean SPEC_DUMPING_MODE = false;

    public static boolean IS_DT_MANUAL_CONTROL = false;
    public static boolean IS_DT_AUTO_ALIGNING = false;
    public static double HOLDPOINT_MANUAL_FEEDFORWARD = 0.5;

    // Size of robot (in inches)
    public static double ROBOT_LENGTH = 12.8;
    public static double ROBOT_WIDTH = 12.8;

    public static double CV_INTAKE_MAX_VELOCITY = 1.0;

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
    public static double INTAKE_ARM_PIVOT_SUBMERSIBLE_SCAN_POS = 0.4;
    public static double INTAKE_ARM_PIVOT_TRANSFER_POS = 0.29;
    public static double INTAKE_ARM_PIVOT_HOVER_WITH_SAMPLE_POS = 0.3; // TODO: update
    public static double INTAKE_ARM_PIVOT_HOVER_INTAKE_POS = 0.65;
    public static double INTAKE_ARM_PIVOT_HOVER_INTAKE_MANUAL_POS = 0.21;
    public static double INTAKE_ARM_PIVOT_INTAKE_POS = 0.11; // 0.79


    // Intake Claw Pivot
    public static double INTAKE_CLAW_PIVOT_TRANSFER_POS = 0.155;
    public static double INTAKE_CLAW_PIVOT_HOVER_INTAKE_POS = 0.735; //0.79
    public static double INTAKE_CLAW_PIVOT_HOVER_INTAKE_MANUAL_POS = 0.615;
    public static double INTAKE_CLAW_PIVOT_INTAKE_POS = 0.53;
    public static double INTAKE_CLAW_PIVOT_HOLDING_POS = 0.45; // TODO: update

    // Intake Claw Rotation
    public static double INTAKE_CLAW_ROTATION_TRANSFER_POS = 0.53;
    public static double INTAKE_CLAW_ROTATION_FULLY_LEFT_POS = 0.875;
    public static double INTAKE_CLAW_ROTATION_FULLY_RIGHT_POS = 0.19;

    // Intake Claw
    public static double INTAKE_CLAW_OPEN_POS = 0.6; //0.49;
    public static double INTAKE_CLAW_MICRO_OPEN_POS = 0.38; //0.775;
    public static double INTAKE_CLAW_CLOSED_POS = 0.365;//0.8;
    public static double INTAKE_CLAW_INTAKING_POS = 0.6/*+0.05+0.025*/;


    // Extendo
    public static int MAX_EXTENDO_EXTENSION = 2120;
    public static int EXTENDO_FEEDFORWARD_TRIGGER_THRESHOLD = 1300;
    public static double INTAKE_MINIMUM_EXTENSION = 2+1.25;

    // 2143 ticks / 21.26 inch --> 100.7996 ticks / 1 inch
    public static double EXTENDO_TICKS_PER_INCH = 100.8;
    // xxx sec / xxx ticks --> xxx sec / xxx ticks
    public static double EXTENDO_MOVEMENT_TIME = 0;

    // Deposit Arm Pivot
    // Bottom servo is 0.01 less than top servo
    // Range (with top servo): 0.75 (wall pickup) --> 0.2 (out the front)
    public static double DEPOSIT_ARM_PIVOT_TRANSFER_POS = 0.315;
    public static double DEPOSIT_ARM_PIVOT_LVL1_ASCENT_POS = 0.29;
    public static double DEPOSIT_ARM_PIVOT_BUCKET_POS = 0.6; //0.65;
    public static double DEPOSIT_ARM_PIVOT_AUTONOMOUS_BUCKET_POS = 0.6;
    public static double DEPOSIT_ARM_PIVOT_SPECIMEN_INTAKE_POS = 0.77;
    public static double DEPOSIT_ARM_PIVOT_SPECIMEN_SCORING_AUTONOMOUS_POS = 0.29;
    public static double DEPOSIT_ARM_PIVOT_HUGGING_SPECIMEN_SCORING_POS = 0.35;
    public static double DEPOSIT_ARM_PIVOT_SPECIMEN_SCORING_TELEOP_POS = 0.35;
    public static double DEPOSIT_ARM_PIVOT_PUSHING_SPECIMEN_POS = 0.34;

    // Deposit Claw Pivot
    // Full range: 0.40 --> 1.0
    public static double DEPOSIT_CLAW_PIVOT_TRANSFER_POS = 0.37;
    public static double DEPOSIT_CLAW_PIVOT_LVL1_ASCENT_POS = 0.8;
    public static double DEPOSIT_CLAW_PIVOT_BUCKET_POS = 0.775; //0.8;
    public static double DEPOSIT_CLAW_PIVOT_AUTONOMOUS_BUCKET_POS = 0.705;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_INTAKE_POS = 0.64; // 0.64
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_SETUP_AUTONOMOUS_POS = 0.55;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_AUTONOMOUS_POS = 0.49;
    public static double DEPOSIT_CLAW_PIVOT_HUGGING_SPECIMEN_SCORING_SETUP_POS = 0.40;
    public static double DEPOSIT_CLAW_PIVOT_HUGGING_SPECIMEN_SCORING_POS = 0.40;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_SETUP_TELEOP_POS = 0.55;
    public static double DEPOSIT_CLAW_PIVOT_SPECIMEN_SCORING_TELEOP_POS = 0.57;
    public static double DEPOSIT_CLAW_PIVOT_PUSHING_SPECIMEN_POS = 0.5;

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.38;
    public static double DEPOSIT_CLAW_OPEN_TRANSFER_POS = 0.47;
    public static double DEPOSIT_CLAW_CLOSED_POS = 0.70;
    public static double DEPOSIT_CLAW_MICRO_OPEN_POS = 0.65;

    // Deposit Claw Rotation
    public static double DEPOSIT_CLAW_ROTATION_TRANSFER_POS = 0.89; //0.3365
    public static double DEPOSIT_CLAW_ROTATION_SPECIMEN_SCORING_POS = 0.3365; //0.3365
    public static double DEPOSIT_CLAW_ROTATION_SAMPLE_OZ_DROP_TELEOP_POS = 0.3365-0.02; //0.87;
    public static double DEPOSIT_CLAW_ROTATION_SAMPLE_OZ_DROP_AUTO_POS = 0.84; //0.3365
    public static double DEPOSIT_CLAW_ROTATION_TELEOP_BUCKET_SCORING_POS = 0.89; //0.615;
    public static double DEPOSIT_CLAW_ROTATION_AUTONOMOUS_BUCKET_SCORING_POS = 0.89; //0.0575
    public static double DEPOSIT_CLAW_ROTATION_LOW_BUCKET_SCORING_POS = 0.615;
    public static double DEPOSIT_CLAW_ROTATION_AUTO_BUCKET_SCORING_POS = 0.89; //0.615
    public static double DEPOSIT_CLAW_ROTATION_SPECIMEN_SCORING_AUTONOMOUS_POS = 0.3365;


    public static int MAX_SLIDES_EXTENSION = 2070;
    public static int HOLDING_SPECIMEN_HEIGHT = 100;
    public static int LOW_BUCKET_HEIGHT = 1000;
    public static int HIGH_BUCKET_HEIGHT = 2000;
    public static int HIGH_BUCKET__AUTO_HEIGHT = 1925;

    public static int LOW_SPECIMEN_HEIGHT = 85;
    public static int HIGH_SPECIMEN_SETUP_AUTONOMOUS_HEIGHT = 885+50;
    public static int HIGH_SPECIMEN_AUTONOMOUS_HEIGHT = 480;
    public static int HIGH_SPECIMEN_HUGGING_SETUP_HEIGHT = 770+30+30+30+20+30+50+25;
    public static int HIGH_SPECIMEN_HUGGING_HEIGHT = 360+30+30+30+20+30-50;
    public static int HIGH_SPECIMEN_SETUP_TELEOP_HEIGHT = 830; // 1000
    public static int HIGH_SPECIMEN_TELEOP_HEIGHT = 1150; // 1350
    public static int PUSHING_SPECIMEN_HEIGHT = 650;
    public static int LVL1_ASCENT_HEIGHT = 500;
    public static int ENDGAME_ASCENT_SETUP_HEIGHT = 2060;
    public static int ENDGAME_ASCENT_HEIGHT = 1110+120+100;
    public static int POST_BUZZER_HANG_RELEASE_HEIGHT = 1550+120-50; // 1600 before, which tilted the robot
}