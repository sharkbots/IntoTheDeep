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

        FollowerConstants.forwardZeroPowerAcceleration = -39.7374294947;
        FollowerConstants.lateralZeroPowerAcceleration = -76.5586676818;

        FollowerConstants.translationalPIDFFeedForward = 0.005;
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.03;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.3,0,0.04,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.5,0,0.18,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.022;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,0.00004,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.04,0,0.00085,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 2;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

//        FollowerConstants.useVoltageCompensationInAuto = true;
//        FollowerConstants.useVoltageCompensationInTeleOp = true;
//        FollowerConstants.nominalVoltage = 12.5;
//        FollowerConstants.cacheInvalidateSeconds = 0.5;

        FollowerConstants.useBrakeModeInTeleOp = false;
    }
}
