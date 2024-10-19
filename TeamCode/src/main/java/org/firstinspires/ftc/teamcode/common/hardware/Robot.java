package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.pathGeneration.MathFunctions;
//import org.firstinspires.ftc.teamcode.common.utils.ConfigMenu;
import org.firstinspires.ftc.teamcode.common.utils.Globals;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Config
public class Robot extends SubsystemWrapper{


    //TODO: ADD COMMENTS TO CLASS

    public HashMap<Sensors.SensorType, Object> sensorValues;

    private ArrayList<SubsystemWrapper> subsystems;
    //public ConfigMenu configMenu;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder strafeEncoder;


    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private volatile double imuYaw = 0;
    private double imuYawOffset = 0;

    private double imuReadTimeIntervalMS = 250;
    long lastIMUReadTimestamp = System.currentTimeMillis();


    private double voltageReadTimeIntervalMS = 5000;
    long lastVoltageReadTimestamp = System.currentTimeMillis();

    private VisionPortal visionPortal;

    private HardwareMap hardwareMap;

    public List<LynxModule> hubs;

    private static Robot instance = null;
    private boolean instantiated;


    /**
     * Allows for only one instance of the robot hardware to be created across all files.
     *
     * @return The sole instance of said robot hardware
     */

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
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
        sensorValues = new HashMap<>();

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        leftEncoder.setDirection(Encoder.FORWARD);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.REVERSE);

        // Retrieve hubs and enable bulk caching
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if(Globals.IS_AUTO){
            //ConfigMenu configMenu = new ConfigMenu();
        }

        synchronized (imuLock){
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        }
        imuYawOffset = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        sensorValues.put(Sensors.SensorType.VOLTAGE, hardwareMap.voltageSensor.iterator().next().getVoltage());


    }

    public void addSubsystem(SubsystemWrapper... subsystems){
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void read(){
        long currentTimeMS = System.currentTimeMillis();

        if (currentTimeMS - lastVoltageReadTimestamp > voltageReadTimeIntervalMS){
            sensorValues.put(Sensors.SensorType.VOLTAGE, hardwareMap.voltageSensor.iterator().next().getVoltage());
            lastVoltageReadTimestamp = currentTimeMS;
        }
    }

    public void write(){}

    public void periodic(){
        if(Globals.IS_AUTO){
            //configMenu.periodic();
        }
    }

    public void reset(){
        for (SubsystemWrapper subsystem : subsystems){
            subsystem.reset();
        }
    }

    public void setBulkCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache(){
        for (LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        imuYawOffset = imuYaw;
    }

    /**
     * Starts a separate thread to continuously update the IMU's yaw angle in radians.
     * This function is useful for keeping track of the robot's orientation while the OpMode is running.
     * The IMU yaw angle is normalized to a value between -π and π radians.
     *
     * @param opMode a LinearOpMode, used to check if the OpMode is still active.
     */
    public void startIMUThread(LinearOpMode opMode){
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuYaw = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                }
            }
        });
        imuThread.start();
    }

    public void updateIMUYaw(){
        imuYaw = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public double getIMUYaw(){
        return MathFunctions.normalizeAngle(imuYaw);
    }

    public double getVoltage(){
        return doubleSubscriber(Sensors.SensorType.VOLTAGE);
    }

    public double doubleSubscriber(Sensors.SensorType topic) {
        Object value = sensorValues.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = sensorValues.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.SensorType topic) {
        Object value = sensorValues.getOrDefault(topic, 0);
        if (value instanceof Boolean) {
            return (Boolean) value;
        } else {
            throw new ClassCastException();
        }
    }

    public void startCamera(){}

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled){
        this.visionPortal.setProcessorEnabled(processor, enabled);
    }

    public VisionPortal.CameraState getCameraState(){
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

    public void closeCamera(){
        if (visionPortal != null) visionPortal.close();
    }

    public void kill(){
        instance = null;
    }

}
