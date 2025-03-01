package org.firstinspires.ftc.teamcode.common.drive.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "dtFrontLeftMotor";
        FollowerConstants.leftRearMotorName = "dtBackLeftMotor";
        FollowerConstants.rightFrontMotorName = "dtFrontRightMotor";
        FollowerConstants.rightRearMotorName = "dtBackRightMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.1;

        FollowerConstants.xMovement = 74.5522337378;
        FollowerConstants.yMovement = 59.4017543365;

        FollowerConstants.forwardZeroPowerAcceleration = -105; // og value is -39.7374294947, changed because multiplier doesn't work
        FollowerConstants.lateralZeroPowerAcceleration = -155; // og value is -76.5586676818, changed because multiplier doesn't work

        FollowerConstants.translationalPIDFFeedForward = 0.005;
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.03;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.45,0,0.03,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.022;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.03,0,0.00005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.drivePIDFSwitch = 22; // This affects the smoothness yes slowness of the deceleration considerably (default: 20)
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.00085,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 2; // does not do anything???
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        //FollowerConstants.useVoltageCompensationInAuto = true;
        //FollowerConstants.useVoltageCompensationInTeleOp = true;

        FollowerConstants.useBrakeModeInTeleOp = false;
    }
}
