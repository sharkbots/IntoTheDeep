package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ActuatorGroupWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.EncoderWrapper;

@Config
@TeleOp(name = "ExtendoActuatorTest")
public class ExtendoActuatorTest extends OpMode {
    private DcMotorEx extendoMotor;
    private EncoderWrapper extendoEncoder;

    private ActuatorGroupWrapper extendoActuator;

    private double loopTime = 0.0;

    private GamepadEx gamepadEx;

    public static int targetPos = 400;
    public static double ekP = 0.000;
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
        telemetry.addData("extendo position", extendoActuator.getPosition() / 26);
        telemetry.addData("extendo power", extendoActuator.getPower());
        loopTime = loop;
        telemetry.update();
    }
}
