package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.Menu.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.utils.Globals;

@Disabled
@TeleOp(name = "A CONFIG MENU")
public class ConfigMenuTest extends LinearOpMode {

    //AutoBase.Coordinates c;
    GamepadEx gamepadEx2;
    ConfigMenu menu;

    public enum QUADRANT {
        CHAMBER_BASKET,
        CHAMBER_MIDDLE,
        CHAMBER_OBSZONE,
        CENTER_BASKET,
        CENTER_MIDDLE,
        CENTER_OBSZONE
    };

    public enum ALLIANCE {
        BLUE,
        RED
    };

    class TEST {

        boolean bool = false;
        int integer = 0;
        float fl = 0.0f;
        double dbl = 0.0;
        QUADRANT quadrant = QUADRANT.CHAMBER_BASKET;
        ALLIANCE alliance = ALLIANCE.BLUE;

    }
    public void Setup() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);
        gamepadEx2 = new GamepadEx(gamepad2);
        menu = new ConfigMenu(gamepadEx2, Robot.getInstance());
        menu.setConfigurationObject(new TEST());

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

