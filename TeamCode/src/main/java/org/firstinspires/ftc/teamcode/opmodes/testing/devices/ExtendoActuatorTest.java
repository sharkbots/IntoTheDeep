package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import android.provider.Settings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ActuatorGroupWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.EncoderWrapper;

@Config
@TeleOp(name = "ExtendoActuatorTest", group="2 tests")
public class ExtendoActuatorTest extends OpMode {
    private DcMotorEx extendoMotor;
    private EncoderWrapper extendoEncoder;

    private ActuatorGroupWrapper extendoActuator;

    private double loopTime = 0.0;

    private GamepadEx gamepadEx;

    public static int targetPos;
    public static double ekP = 0.005;
    public static double ekI = 0.0;
    public static double ekD = 0.0;
    public static int lTolerance = 20;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendoMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);
        extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoEncoder = new EncoderWrapper(new MotorEx(hardwareMap, "extendoMotor").encoder);
        extendoEncoder.setDirection(EncoderWrapper.EncoderDirection.FORWARD);

        this.extendoActuator = new ActuatorGroupWrapper(extendoEncoder, extendoMotor)
                .setPIDController(new PIDController(ekP, ekI, ekD))
                .setFeedforward(ActuatorGroupWrapper.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                .setErrorTolerance(lTolerance);
        extendoActuator.read();
    }

    @Override
    public void loop() {
        extendoActuator.updatePID(ekP, ekI, ekD);
        extendoActuator.read();
        extendoActuator.setTargetPosition(targetPos);
        extendoActuator.periodic();
        extendoActuator.write();

        telemetry.addData("extendo ticks", extendoEncoder.getPosition());
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("extendo position", extendoActuator.getPosition() / Globals.EXTENDO_TICKS_PER_INCH);
        telemetry.addData("extendo power", extendoActuator.getPower());
        loopTime = loop;
        telemetry.update();
    }
}
