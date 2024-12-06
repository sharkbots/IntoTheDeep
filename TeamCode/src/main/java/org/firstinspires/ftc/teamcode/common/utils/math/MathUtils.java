package org.firstinspires.ftc.teamcode.common.utils.math;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class MathUtils {
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static boolean epsilonEquals(double val1, double val2) {
        return Math.abs(val1 - val2) < 1e-6;
    }

    public static Vector2d toCartesian(double r, double theta) {
        return new Vector2d(r * Math.cos(theta), r * Math.sin(theta));
    }

    public static double getRadRotDist(double start, double end){
        double diff = (end - start + Math.PI) % (2 * Math.PI) - Math.PI;
        return diff < -Math.PI ? (diff + (Math.PI * 2)) : diff;
    }

    public static double getRotDist(double start, double end){
        return MathUtils.getRadRotDist(start, end);
    }

//    public static double joystickScalar(double num, double min) {
//        return joystickScalar(num, min, 0.1, 3.5);
//    }

    public static double joystickScalar(double joystick, double minPower, double deadzone, double scale) {
        if (joystick<0) minPower *= -1;
        if (Math.abs(joystick) < deadzone) return 0;

        return joystick + Math.pow(Math.pow((1-minPower), (1/scale)-1), scale)+minPower;
    }


}