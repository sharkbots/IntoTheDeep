package org.firstinspires.ftc.teamcode.opmodes.testing.devices;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.utils.wrappers.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ServoWrapper;

import java.util.ArrayList;

@Config
@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
    private ServoWrapper intakeArmPivotLeftServo, intakeArmPivotRightServo, intakeClawPivotServo,
            intakeClawServo, intakeClawRotationServo;
    private ServoWrapper depositPivotServo, depositClawServo, depositClawRotationServo;

    private ArrayList<ServoWrapper> servos = new ArrayList<>();

    public AnalogInput intakeArmPivotLeftEnc, intakeArmPivotRightEnc, intakeClawPivotEnc;
    public AbsoluteAnalogEncoder intakeArmPivotLeftEncoder, intakeArmPivotRightEncoder, intakeClawPivotEncoder;

    private ArrayList<AbsoluteAnalogEncoder> analogEncoders = new ArrayList<>();

    public static double incrementStep = 0.05;
    public static double targetPos = 0.0;
    public static int servoID = 0;

    private static boolean holdPosition = false;

    GamepadEx gamepadEx;

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        intakeArmPivotLeftServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeArmPivotLeftServo"));
        servos.add(intakeArmPivotLeftServo);

        intakeArmPivotRightServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeArmPivotRightServo"));
        intakeArmPivotRightServo.setDirection(Servo.Direction.REVERSE);
        servos.add(intakeArmPivotRightServo);

        intakeClawPivotServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawPivotServo"));
        servos.add(intakeClawPivotServo);

        intakeClawServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawServo"));
        servos.add(intakeClawServo);

        intakeClawRotationServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawRotationServo"));
        servos.add(intakeClawRotationServo);

        depositPivotServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositPivotServo"));
        servos.add(depositPivotServo);

        depositClawServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositClawServo"));
        servos.add(depositClawServo);

        depositClawRotationServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositClawRotationServo"));
        servos.add(depositClawRotationServo);

        intakeArmPivotLeftEnc = hardwareMap.analogInput.get("intakeArmPivotLeftEncoder");
        intakeArmPivotLeftEncoder = new AbsoluteAnalogEncoder(intakeArmPivotLeftEnc)
                .zero(0.0)
                .setWraparound(false);
        analogEncoders.add(intakeArmPivotLeftEncoder);

        intakeArmPivotRightEnc = hardwareMap.analogInput.get("intakeArmPivotRightEncoder");
        intakeArmPivotRightEncoder = new AbsoluteAnalogEncoder(intakeArmPivotRightEnc)
                .zero(0.0)
                .setInverted(true)
                .setWraparound(false);
        analogEncoders.add(intakeArmPivotRightEncoder);

        intakeClawPivotEnc = hardwareMap.analogInput.get("intakeClawPivotEncoder");
        intakeClawPivotEncoder = new AbsoluteAnalogEncoder(intakeClawPivotEnc)
                .zero(0.0)
                .setWraparound(false);
        analogEncoders.add(intakeClawPivotEncoder);

        for (ServoWrapper servo : servos) {
            servo.resolveName(hardwareMap);
        }

        for (AbsoluteAnalogEncoder enc : analogEncoders){
            enc.resolveName(hardwareMap);
        }
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();
        // Toggle caching position mode
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            holdPosition = !holdPosition;
        }

        // Cycle through servos with X/Y buttons
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
            if (!holdPosition) {
                // Disable PWM for the current servo if hold position is disabled
                servos.get(servoID).setPwmDisable();
            }
            servoID = (servoID + 1) % servos.size(); // Increment and loop around
            servos.get(servoID).setPwmEnable(); // Enable PWM for the new servo
            //targetPos = 0.0;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
            if (!holdPosition) {
                // Disable PWM for the current servo if hold position is disabled
                servos.get(servoID).setPwmDisable();
            }
            servoID = (servoID - 1 + servos.size()) % servos.size(); // Decrement and loop around
            servos.get(servoID).setPwmEnable(); // Enable PWM for the new servo
            //targetPos = 0.0;
        }

        // Adjust servo position with Left/Right bumpers
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            targetPos = Math.max(0.00, targetPos-incrementStep); // Decrease position but don't go below 0
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            targetPos = Math.min(1.00, targetPos+incrementStep); // Increase position but don't exceed 1
        }

        // Set the current servo to the target position
        servos.get(servoID).setPosition(targetPos);

        // Display telemetry
        telemetry.addData("Servo", servos.get(servoID).getConfigName());
        telemetry.addData("Position", targetPos);
        telemetry.addData("Holding position", holdPosition);

        for (AbsoluteAnalogEncoder enc : analogEncoders){
            telemetry.addData("Analog encoder", enc.getConfigName());
            telemetry.addData("Analog position", enc.getCurrentPosition());
        }
        telemetry.update();
    }
}
