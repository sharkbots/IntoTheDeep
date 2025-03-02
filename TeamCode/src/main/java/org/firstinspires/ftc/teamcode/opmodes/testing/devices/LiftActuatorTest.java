package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ActuatorGroupWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.EncoderWrapper;

@Config
@TeleOp(name = "LiftActuatorTest")
public class LiftActuatorTest extends OpMode {
    private DcMotorEx liftBottomMotor, liftCenterMotor, liftTopMotor;
    private EncoderWrapper liftBottomEncoder, liftCenterEncoder, liftTopEncoder;
    private ActuatorGroupWrapper liftActuator;

    private double loopTime = 0.0;

    private GamepadEx gamepadEx;

    public static int targetPos = 400;
    public static double lkP = 0.005;
    public static double lkI = 0.0;
    public static double lkD = 0.0;
    public static int lTolerance = 20;
    public static double feedForward = 0.20;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        liftBottomMotor = hardwareMap.get(DcMotorEx.class, "liftBottomMotor");
        liftBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftBottomMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);
        liftBottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        liftCenterMotor = hardwareMap.get(DcMotorEx.class, "liftCenterMotor");
        liftCenterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftCenterMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);

        liftTopMotor = hardwareMap.get(DcMotorEx.class, "liftTopMotor");
        liftTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftTopMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);
        liftTopMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.liftTopEncoder = new EncoderWrapper(new MotorEx(hardwareMap, "liftCenterMotor").encoder);
        liftTopEncoder.setDirection(EncoderWrapper.EncoderDirection.FORWARD);


        this.liftActuator = new ActuatorGroupWrapper(liftTopEncoder, liftTopMotor, liftCenterMotor, liftBottomMotor)
                .setPIDController(new PIDController(lkP, lkI, lkD))
                .setFeedforward(ActuatorGroupWrapper.FeedforwardMode.CONSTANT, feedForward)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                .setErrorTolerance(lTolerance);
        liftActuator.read();
    }

    @Override
    public void loop() {
        liftActuator.updatePID(lkP, lkI, lkD);
        liftActuator.updateFeedforward(feedForward);
        liftActuator.read();
        liftActuator.setTargetPosition(targetPos);
        liftActuator.periodic();
        liftActuator.write();

        double liftTicks = liftActuator.getPosition();
        double targetTickPos = liftActuator.getTargetPosition();
        double tickError = Math.abs(liftTicks-targetTickPos);

        telemetry.addData("lift ticks", liftTicks);
        telemetry.addData("lift target pos", targetTickPos);
        telemetry.addData("lift tick error", tickError);
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("lift position", liftActuator.getPosition() / 26);
        telemetry.addData("lift power", liftActuator.getPower());
        loopTime = loop;
        telemetry.update();
    }
}
