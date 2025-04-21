package org.firstinspires.ftc.teamcode.opmodes.testing;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.GRABBING_MODES;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.IS_AUTONOMOUS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.utils.Globals;

@Disabled
@TeleOp(name = "🎮🎮🎮 button tests", group = "1 Teleop")
public class PS5_Buttons_Tests extends CommandOpMode {

    private DashboardPoseTracker dashboardPoseTracker;

    //private final Robot robot = Robot.getInstance();

    private GamepadEx driver;
    private GamepadEx operator;

    private PIDController dtHeadingLockOn;

    private double headingLockTolerance = Math.toRadians(5);
    private boolean flicking = false;


    private double loopTime = 0.0;
    private ElapsedTime timer;

    public static double dtMinPower = 0.1;
    public static double dtDeadzone = 0.1;
    public static double dtScale = 3.5;

    private boolean notifiedEndgame = false;
    private boolean hangDone = false;
    private boolean readyToLetGo = false;

    private int matchLength = 120;
    private int increaseCounter = 0;


    @Override
    public void initialize() {
        super.reset();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        IS_AUTONOMOUS = false;
        Globals.GRABBING_MODES.set(GRABBING_MODES.SAMPLE);
        UpdateOperatorGamepadColor();

        operator.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
                .whenPressed(
                        new InstantCommand(() -> {
                            GRABBING_MODES.next();
                            UpdateOperatorGamepadColor();
                        })
                );



        super.run();
        while (opModeInInit()) {

        }
    }

    private void UpdateOperatorGamepadColor() {
        int[] rgb = GRABBING_MODES.getControllerColor();
        gamepad2.setLedColor(rgb[0], rgb[1], rgb[2], LED_DURATION_CONTINUOUS);
    }


    @Override
    public void run(){
        //  Runs FTCLib Command Scheduler
        super.run();


    }

    @Override
    public void reset(){
        super.reset();
    }
}
