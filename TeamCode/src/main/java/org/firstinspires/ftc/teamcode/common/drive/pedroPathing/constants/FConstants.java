package org.firstinspires.ftc.teamcode.common.drive.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
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

        FollowerConstants.mass = 13.52; //29.8 lbs

        FollowerConstants.xMovement = 73.1688454389;
        FollowerConstants.yMovement = 55.823867805;

        FollowerConstants.forwardZeroPowerAcceleration = -35.9396675102;
        FollowerConstants.lateralZeroPowerAcceleration = -77.189323481;

        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.useSecondaryDrivePID = true;


        FollowerConstants.translationalPIDFFeedForward = 0.02; //0.03
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);

        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.03; //0.01
        // 0.075 p
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.2,0,0.1,0); // Not being used, @see useSecondaryTranslationalPID

        // 5p 0.5d
        FollowerConstants.headingPIDFCoefficients.setCoefficients(5,0,0.3,0);
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.02; //0.01
        // 1.5p 0.1d
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.15,0); // Not being used, @see useSecondaryHeadingPID


        FollowerConstants.drivePIDFFeedForward = 0.01;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0001,0.6,0);
        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,0.00004,0.6,0);


        FollowerConstants.secondaryDrivePIDFFeedForward = 0.01;

        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.0005,0.6,0);
        //FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.04,0,0.00085,0.6,0);


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
