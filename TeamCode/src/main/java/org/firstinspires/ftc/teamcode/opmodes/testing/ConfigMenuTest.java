package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utils.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

@TeleOp(name = "A CONFIG MENU")
public class ConfigMenuTest extends LinearOpMode {

    //AutoBase.Coordinates c;
    GamepadEx gamepadEx2;
    ConfigMenu menu;

    class TEST {
        boolean bool = false;
        int integer = 0;
        float fl = 0.0f;
        double dbl = 0.0;
    }
    public void Setup() {
        telemetry = telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);
        gamepadEx2 = new GamepadEx(gamepad2);
        menu = new ConfigMenu();
        menu.setConfigurationObject(new TEST());
        menu.setGamepad(gamepadEx2);

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
            telemetry.addLine("<!doctype html><html><head><title>Logs</title></head><body>" +
                    "<b>TEST </b><i>TEST</i><p style=\"color:yellow\">GREEN</p></body></html>");
            menu.periodic();
        }

    }
}

 */