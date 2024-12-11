package org.firstinspires.ftc.teamcode.common.utils;

import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Pose;

public class Globals {
    public enum ALLIANCE{
        BLUE,
        RED
    }

    public static ALLIANCE ALLIANCE = Globals.ALLIANCE.RED;
    public static boolean IS_AUTO = false;
    public static boolean COMING_FROM_AUTONOMOUS = false;
    public static boolean IS_FIELD_CENTRIC = true;
    public static boolean INTAKING = false;

    public static Pose END_OF_AUTO_POSE = new Pose();
}
