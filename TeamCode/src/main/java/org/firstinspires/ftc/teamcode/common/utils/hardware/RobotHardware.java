package org.firstinspires.ftc.teamcode.common.utils.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

@Config
public class RobotHardware {

    private HardwareMap hardwareMap;

    private static RobotHardware instance = null;
    private boolean instantiated;

    public HashMap<Sensors.SensorType, Object> values;

    /**
     * Allows for only one instance of the hardware to be created across all files.
     *
     * @return RobotHardware instance
     */

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.instantiated = true;
        return instance;
    }


    /**
     * Ran once at the start of every OpMode.
     * Instantiates and configures all robot hardware.
     *
     * @param hardwareMap The robot's HardwareMap containing all hardware devices
     */
    public void init(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        values = new HashMap<>();


    }

}
