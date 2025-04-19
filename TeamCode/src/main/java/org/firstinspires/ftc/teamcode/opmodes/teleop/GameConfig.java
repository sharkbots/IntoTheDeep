package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;

@TeleOp(name = "game config", group="1 Teleop")
public class GameConfig extends LinearOpMode {

    //AutoBase.Coordinates c;
    GamepadEx gamepadEx2;
    ConfigMenu menu;
    Robot robot = Robot.getInstance();

    public void Setup() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.setTelemetry(telemetry);
        menu = new ConfigMenu(gamepadEx2, robot);
        menu.setConfigurationObject(new Globals.SampleAutonomousConfig());

        while(!isStarted() && !isStopRequested()){
        }
        Globals.COMING_FROM_AUTONOMOUS = false;
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(menu::backupCurrentField);

    }

    @Override
    public void runOpMode() {
        Setup();
        waitForStart();
        while(opModeIsActive()){
          //  telemetry.addLine("<!doctype html><html><head><title>Logs</title></head><body>" +
          //          "<b>TEST </b><i>TEST</i><p style=\"color:yellow\">GREEN</p></body></html>");
            menu.periodic();
        }

    }
}

