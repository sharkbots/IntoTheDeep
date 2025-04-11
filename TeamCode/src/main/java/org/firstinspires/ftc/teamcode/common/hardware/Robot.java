package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.common.drive.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.LiftSubsystem;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.*;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.common.utils.wrappers.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ActuatorGroupWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.EncoderWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.ServoWrapper;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;
import org.firstinspires.ftc.teamcode.common.vision.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

//TODO: ADD COMMENTS TO CLASS
@Config
public class Robot extends SubsystemWrapper{

    // drivetrain
    public DcMotorEx dtBackLeftMotor, dtBackRightMotor, dtFrontLeftMotor, dtFrontRightMotor;


    // extendo
    public DcMotorEx extendoMotor;

    public EncoderWrapper extendoEncoder;

    public ActuatorGroupWrapper extendoActuator;


    // lift
    public DcMotorEx liftBottomMotor, liftCenterMotor, liftTopMotor;

    public EncoderWrapper liftTopEncoder;

    public ActuatorGroupWrapper liftActuator;


    // intake
    public ServoImplEx intakeClawLED;
    public ServoWrapper intakeArmPivotLeftServo, intakeArmPivotRightServo, intakeClawPivotServo, intakeClawServo, intakeClawRotationServo;

    public AnalogInput intakeArmPivotLeftEnc, intakeArmPivotRightEnc, intakeClawPivotEnc;
    public AbsoluteAnalogEncoder intakeArmPivotLeftEncoder, intakeArmPivotRightEncoder, intakeClawPivotEncoder;


    public ActuatorGroupWrapper intakeArmPivotActuator, depositArmPivotActuator /*intakeClawPivotActuator*/;

    // deposit
    public ServoWrapper depositArmPivotTopServo, depositArmPivotBottomServo, depositClawPivotServo, depositClawServo, depositClawRotationServo;

    public AnalogInput depositPivotEnc;
    public AbsoluteAnalogEncoder depositPivotEncoder;

    // Subsystems
    private ArrayList<SubsystemWrapper> subsystems;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public MecanumDrivetrain drivetrain;

    // Pedro Pathing
    public Follower follower;
    public Pose previousPose = new Pose(0, 0, 0);

    // IMU
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private volatile double imuYaw = 0;
    public double imuYawOffset = 0;
    private double imuYawStartOffset = 0;
    private double imuReadTimeIntervalMS = 250;
    long lastIMUReadTimestamp = System.currentTimeMillis();


    public Telemetry telemetryA;

    // vision
    public Vision vision;


    private final double voltageReadTimeIntervalMS = 5000;
    long lastVoltageReadTimestamp = System.currentTimeMillis();

    private HardwareMap hardwareMap;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


    public HashMap<Sensors.SensorType, Object> sensorValues;

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
        this.sensorValues = new HashMap<>();

        sensorValues.put(Sensors.SensorType.VOLTAGE, 0);
        sensorValues.put(Sensors.SensorType.EXTENDO_ENCODER, 0);
        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_LEFT_ENCODER, 0.0);
        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_RIGHT_ENCODER, 0.0);
        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_ROTATION_ENCODER, 0.0);
        sensorValues.put(Sensors.SensorType.LIFT_BOTTOM_ENCODER, 0);
        sensorValues.put(Sensors.SensorType.LIFT_CENTER_ENCODER, 0);
        sensorValues.put(Sensors.SensorType.LIFT_TOP_ENCODER, 0);
        sensorValues.put(Sensors.SensorType.DEPOSIT_PIVOT_ENCODER, 0.0);

        // Vision
        vision = new Vision();
        //vision.cvCamera.setWebcamName(hardwareMap.get(WebcamName.class, "Webcam 1"));
        vision.limelight.setLimelight(hardwareMap.get(Limelight3A.class, "Limelight3A"));
        vision.limelight.setPipeline(7);
        vision.limelight.start();
//        if (IS_AUTONOMOUS){
//            vision.limelight.setPipeline(7);
//            vision.limelight.start();
//        }

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // EXTENDO
        this.extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendoMotor.setCurrentAlert(9.2, CurrentUnit.AMPS);
        extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.extendoEncoder = new EncoderWrapper(new MotorEx(hardwareMap, "extendoMotor").encoder);
        extendoEncoder.setDirection(EncoderWrapper.EncoderDirection.FORWARD);

        double ekP = 0.005;
        double ekI = 0.0;
        double ekD = 0.0;
        int eTolerance = 20;
        this.extendoActuator = new ActuatorGroupWrapper(
                () -> intSubscriber(Sensors.SensorType.EXTENDO_ENCODER), extendoMotor)
                .setPIDController(new PIDController(ekP, ekI, ekD))
                .setFeedforward(ActuatorGroupWrapper.FeedforwardMode.CONSTANT, 0.0)
//                .setMotionProfile(0, new ProfileConstraints(1000, 5000, 2000))
                .setErrorTolerance(eTolerance)
                .setMinPos(0)
                .setMaxPos(MAX_EXTENDO_EXTENSION);

        // INTAKE

        intakeArmPivotLeftServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeArmPivotLeftServo"));
        intakeArmPivotLeftServo.setDirection(ServoWrapper.Direction.REVERSE);
        intakeArmPivotLeftServo.setOffset(0.99);


        intakeArmPivotRightServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeArmPivotRightServo"));

        intakeClawPivotServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawPivotServo"));

        intakeClawServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawServo"));

        intakeClawRotationServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("intakeClawRotationServo"));


        double intakeArmPivotTolerance = 0.0;
        this.intakeArmPivotActuator = new ActuatorGroupWrapper(intakeArmPivotLeftServo, intakeArmPivotRightServo)
                .setErrorTolerance(intakeArmPivotTolerance);

        // LIFT
        liftBottomMotor = hardwareMap.get(DcMotorEx.class, "liftBottomMotor");
        liftBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftBottomMotor.setCurrentAlert(1.5, CurrentUnit.AMPS);
        liftBottomMotor.isOverCurrent();
        liftBottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        liftCenterMotor = hardwareMap.get(DcMotorEx.class, "liftCenterMotor");
        liftCenterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftCenterMotor.setCurrentAlert(1.5, CurrentUnit.AMPS);

        liftTopMotor = hardwareMap.get(DcMotorEx.class, "liftTopMotor");
        liftTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftTopMotor.setCurrentAlert(1.5, CurrentUnit.AMPS);
        liftTopMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.liftTopEncoder = new EncoderWrapper(new MotorEx(hardwareMap, "liftCenterMotor").encoder);
        liftTopEncoder.setDirection(EncoderWrapper.EncoderDirection.FORWARD);
//        if(IS_AUTONOMOUS) {
//            liftTopEncoder.reset();
//        }

        double lkP = 0.005;
        double lkI = 0.05;
        double lkD = 0.0;
        int lTolerance = 20;
        this.liftActuator = new ActuatorGroupWrapper(
                () -> intSubscriber(Sensors.SensorType.LIFT_TOP_ENCODER), liftTopEncoder, liftTopMotor, liftCenterMotor, liftBottomMotor)
                .setPIDController(new PIDController(lkP, lkI, lkD))
                .setFeedforward(ActuatorGroupWrapper.FeedforwardMode.CONSTANT, 0.25)
                .setErrorTolerance(lTolerance)
                .setMinPos(0)
                .setMaxPos(MAX_SLIDES_EXTENSION);

        // DEPOSIT
        depositArmPivotTopServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositArmPivotTopServo"));
        depositArmPivotBottomServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositArmPivotBottomServo"));
        //depositArmPivotBottomServo.setOffset(0.01);

        depositClawPivotServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositClawPivotServo"));

        depositClawServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositClawServo"));

        depositClawRotationServo = new ServoWrapper((ServoImplEx) hardwareMap.servo.get("depositClawRotationServo"));

        double depositPivotTolerance = 0.0;
        this.depositArmPivotActuator = new ActuatorGroupWrapper(depositArmPivotTopServo, depositArmPivotBottomServo)
                .setErrorTolerance(depositPivotTolerance);



        // Retrieve hubs and enable bulk caching
        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules){
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;

        }

        subsystems = new ArrayList<>();
        intake = new IntakeSubsystem();
        lift = new LiftSubsystem();
        addSubsystem(intake, lift);

        if (IS_AUTONOMOUS){
            vision.limelight.start();
        }

//        if (IS_AUTONOMOUS) {
//            // TODO: CATCH EXCEPTION TO NOT INIT CAMERA IF ITS NOT FOUND
//            try {
//                vision.cvCamera.startCamera();
//                vision.cvCamera.resumeStreaming();
//                vision.cvCamera.setAutoCameraControls();
//
//                try {
//                    sleep(1000);
//                } catch (Exception e) {
//
//                }
//                vision.cvCamera.setAutoCameraControls();
//            } catch (Exception e) {
//
//            }
//        }


//        if (!Globals.IS_AUTO) {
//            drivetrain = new MecanumDrivetrain();
//            addSubsystem(drivetrain);
//        }

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        if(!IS_AUTONOMOUS){
            //follower.setStartingPose(END_OF_AUTO_POSE);
            follower.setPose(END_OF_AUTO_POSE);
            this.previousPose = END_OF_AUTO_POSE;
        }

//        synchronized (imuLock){
//            imu = hardwareMap.get(IMU.class, "imu");
//            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
//        }
//        imuYawOffset = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        sensorValues.put(Sensors.SensorType.VOLTAGE, hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public void addSubsystem(SubsystemWrapper... subsystems){
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public void read(){
        // Read all hardware devices here
        long currentTimeMS = System.currentTimeMillis();

        if (currentTimeMS - lastVoltageReadTimestamp > voltageReadTimeIntervalMS){
            sensorValues.put(Sensors.SensorType.VOLTAGE, hardwareMap.voltageSensor.iterator().next().getVoltage());
            lastVoltageReadTimestamp = currentTimeMS;
        }
        sensorValues.put(Sensors.SensorType.EXTENDO_ENCODER, extendoEncoder.getPosition());
//        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_LEFT_ENCODER, intakeArmPivotLeftEncoder.getCurrentPosition());
//        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_RIGHT_ENCODER, intakeArmPivotRightEncoder.getCurrentPosition());
//        sensorValues.put(Sensors.SensorType.INTAKE_PIVOT_ROTATION_ENCODER, intakeClawPivotEncoder.getCurrentPosition());
        sensorValues.put(Sensors.SensorType.LIFT_TOP_ENCODER, liftTopEncoder.getPosition());

        for (SubsystemWrapper subsystem : subsystems){
            subsystem.read();
        }
    }

    public void write(){
        for (SubsystemWrapper subsystem : subsystems){
            subsystem.write();
        }
    }

    public void periodic(){
        for (SubsystemWrapper subsystem : subsystems){
            subsystem.periodic();
        }
    }

    public void reset(){
        for (SubsystemWrapper subsystem : subsystems){
            subsystem.reset();
        }

        //imuYawOffset = imuYaw;
    }

    public void setTelemetry(Telemetry telemetry){
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setBulkCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule hub : modules){
            hub.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache(){
        for (LynxModule hub : modules){
            hub.clearBulkCache();
        }
        imuYawOffset = imuYaw;
    }

    public void clearChubCache(){
        CONTROL_HUB.clearBulkCache();
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
        return MathFunctions.normalizeAngle(imuYaw - imuYawOffset + imuYawStartOffset);
    }

    public void setIMUStartOffset(double off) {
        imuYawStartOffset = off;
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

    public void kill(){
        instance = null;
    }
}