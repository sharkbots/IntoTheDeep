package org.firstinspires.ftc.teamcode.common.drive.pedroPathing.util.profile;

public class ProfileState {
    public double x = 0;
    public double v = 0;
    public double a = 0;

    public ProfileState(double x, double v, double a) {
        this.x = x;
        this.v = v;
        this.a = a;
    }

    public ProfileState() {}
}