package org.firstinspires.ftc.teamcode.common.utils.math;

import com.seattlesolvers.solverslib.geometry.Vector2d;

public class MathUtils {
    public static double distance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));
    }

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

    public static double extendoJoystickScalar(double joystick, double deadzone, double linearThreshold1, double linearFactor1, double linearThreshold2,
                                               double linearFactor2, double exponent, double maxPower) {

        double absInput = Math.abs(joystick);
        double output;

        if (absInput < deadzone) {
            output = 0.0;
        } else if (absInput < linearThreshold1) {
            // First linear region
            output = linearFactor1 * maxPower * absInput;
        } else if (absInput < linearThreshold2) {
            // Second linear region
            double baseOutput = linearFactor1 * maxPower * linearThreshold1;
            output = baseOutput + linearFactor2 * maxPower * (absInput - linearThreshold1);
        } else {
            // Curved region polynomial interpolation
            double baseOutput = linearFactor1 * maxPower * linearThreshold1
                    + linearFactor2 * maxPower * (linearThreshold2 - linearThreshold1);
            double t = (absInput - linearThreshold2) / (1 - linearThreshold2);
            output = baseOutput + (maxPower - baseOutput) * Math.pow(t, exponent);
        }

        return Math.copySign(output, joystick);
    }

    public static double joystickScalar(double joystick, double minPower, double deadzone, double scale) {
        if (joystick<0) minPower *= -1;
        if (Math.abs(joystick) < deadzone) return 0;

        return joystick + Math.pow(Math.pow((1-minPower), (1/scale)-1), scale)+minPower;
    }

    /**
     * Scales the right stick x input from [-1.0, 1.0] to [0, 2π].
     *
     * @param rot The raw x-axis input from the right stick (-1.0 to 1.0).
     * @return The scaled value between 0 and 2π.
     */
    public static double rotationJoystickScalar(double rot){
        // Clamp the input value to ensure it's within the range [-1, 1]
        rot = Math.max(-1.0, Math.min(1.0, rot));

        // Scale using the formula
        return (rot + 1) * Math.PI;
    }


}