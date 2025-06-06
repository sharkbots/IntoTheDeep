package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import androidx.core.math.MathUtils;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.utils.math.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.utils.math.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.utils.math.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ActuatorGroupWrapper {
    public enum FeedforwardMode {
        NONE,
        CONSTANT,
        ANGLE_BASED,
        ANGLE_BASED_SIN
    }

    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    private DoubleSupplier voltage;
    public ElapsedTime timer;
    private Robot robot = Robot.getInstance();

    private double position = 0.0;
    private double pTargetPosition = 0.0;
    private double targetPosition = 0.0;
    private double overallTargetPosition = 0.0;
    private double pPower = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforwardMin = 0.0;
    private double feedforwardMax = 0.0;
    private double currentFeedforward = 0.0;
    private double targetPositionOffset = 0.0;
    private double offset = 0.0;
    private int minPos = 0;
    private int maxPos = 100000;

    private boolean reached = false;
    private boolean floating = false;

    private boolean manualMode = false;

    private FeedforwardMode mode = FeedforwardMode.NONE;

    private Sensors.SensorType sensorType;
    private Supplier<Object> topic;

    /**
     * Actuator constructor with varargs HardwareDevice parameter
     *
     * @param devices HardwareDevice objects
     */
    public ActuatorGroupWrapper(HardwareDevice... devices) {
        this.topic = null;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }

    /**
     * Actuator constructor with a topic and varargs HardwareDevice parameter
     * @param topic Supplier object
     * @param devices HardwareDevice objects
     */
    public ActuatorGroupWrapper(Supplier<Object> topic, HardwareDevice... devices) {
        this.topic = topic;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }

    /**
     * Reads every given hardware device containing either AnalogEncoder or Encoder object.
     * Will then store this, and terminate the loop, because there is only a need for one value in
     * a given actuation group.
     */
    public void read() {

        if (topic != null) {
            Object value = topic.get();
            if (value instanceof Integer) {
                this.position = (int) value;
                return;
            } else if (value instanceof Double) {
                this.position = (double) value;
                return;
            }
        }

        for (HardwareDevice device : devices.values()) {
            if (device instanceof AbsoluteAnalogEncoder) {
                this.position = ((AbsoluteAnalogEncoder) device).getCurrentPosition() + offset;
                return;
            } else if (device instanceof EncoderWrapper) {
                this.position = (int) (((EncoderWrapper) device).getPosition() + offset);
                return;
            }
        }
        this.position = 0.0;
    }

    /**
     * Performs arithmetic with the AsymmetricMotionProfile and PIDController.
     * Stores a boolean representing whether or not the actuator group is within
     * some tolerance given by a specified value.
     */
    public void periodic() {
        if (manualMode) {
            this.targetPosition = this.position;
            return;
        }

        if (timer == null) {
            timer = new ElapsedTime();
        }

        if (profile != null) {
            this.state = profile.calculate(timer.time());
            this.targetPosition = state.x + targetPositionOffset;
        }

        if (controller != null) {
            this.power = controller.calculate(position, targetPosition + targetPositionOffset);

            switch (mode) {
                case CONSTANT:
                    this.power += currentFeedforward;
                    break;
                case ANGLE_BASED:
                    this.power -= Math.cos(targetPosition + targetPositionOffset) * currentFeedforward;
                    break;
                case ANGLE_BASED_SIN:
                    this.power -= Math.sin(targetPosition + targetPositionOffset) * currentFeedforward;
                    break;
                default:
            }
            this.power = MathUtils.clamp(power, -1, 1);
        }

        this.reached = Math.abs((targetPosition + targetPositionOffset) - position) < tolerance;
    }

    /**
     * For cohesiveness in our program's sequence, writes back all of the new
     * values calculated and saved. Runs different methods based on the given actuation group.
     */
    public void write() {
        if (manualMode){
            for (HardwareDevice device : devices.values()){
                if (device instanceof DcMotor) {
                    ((DcMotor) device).setPower(power);
                }
            }
        }
        else if (Math.abs(targetPosition - pTargetPosition) > 0.005 ||
                Math.abs(power - pPower) > 0.005) {
            for (HardwareDevice device : devices.values()) {
                if (device instanceof DcMotor) {
                    double correction = 1.0;
                    if (voltage != null) correction = 12.0 / voltage.getAsDouble();
                    if (!floating) ((DcMotor) device).setPower(power * correction);
                    else ((DcMotor) device).setPower(0);
                    pPower = power;
                } else if (device instanceof Servo) {
                    ((Servo) device).setPosition(targetPosition);
                    pTargetPosition = targetPosition;
                }
            }
        }
    }

    public void enableManualPower(){
        manualMode = true;
    }
    public void disableManualPower(){
        manualMode = false;
    }

    public void setManualPower(double power){
        if (position - minPos <= tolerance && power < 0){
            power = 0;
        }
        if (maxPos - position <= tolerance && power > 0){
            power = 0;
        }
        this.power = power;
    }

    public void setOverridePower(double power){
        this.power = power;
    }


    public ActuatorGroupWrapper setMinPos(int pos){
        this.minPos = pos;
        return this;
    }

    public ActuatorGroupWrapper setMaxPos(int pos){
        this.maxPos = pos;
        return this;
    }

    /**
     * Will set the target position for the actuator group. In the case of a motion profile
     * being used, the profile will be reset and created again with the new target position.
     * Otherwise, a PIDController will be used in the periodic() method above.
     *
     * @param targetPosition The target position for the actuator group.
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        this.overallTargetPosition = targetPosition;
    }

    /**
     * Sets the floating property for the actuator group.
     * @param f
     */
    public void setFloat(boolean f) {
        this.floating = f;
    }

    /**
     * Sets the target position offset for the actuator group.
     * @param offset The target position offset.
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setMotionProfileTargetPosition(double targetPosition) {
        this.overallTargetPosition = targetPosition;
        this.profile = new AsymmetricMotionProfile(getTargetPosition(), targetPosition, constraints);
        this.timer.reset();
    }

    public ActuatorGroupWrapper setVoltageSupplier(DoubleSupplier voltage) {
        this.voltage = voltage;
        return this;
    }

    public ActuatorGroupWrapper setMotionProfile(double targetPosition, ProfileConstraints constraints) {
        this.constraints = constraints;
        this.profile = new AsymmetricMotionProfile(position, targetPosition, constraints);
        return this;
    }

    public ActuatorGroupWrapper setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public void setCurrentPosition(double position) {
        this.position = position;

    }

    public ActuatorGroupWrapper setPID(double p, double i, double d) {
        if (controller == null) {
            this.controller = new PIDController(p, i, d);
        } else {
            this.controller.setPID(p, i, d);
        }

        return this;
    }

    public ActuatorGroupWrapper setFeedforward(FeedforwardMode mode, double feedforward) {
        this.mode = mode;
        this.feedforwardMin = feedforward;
        this.currentFeedforward = feedforwardMin;
        return this;
    }

    public ActuatorGroupWrapper setFeedforward(FeedforwardMode mode, double feedforwardMin, double feedforwardMax) {
        this.mode = mode;
        this.feedforwardMin = feedforwardMin;
        this.feedforwardMax = feedforwardMax;
        this.currentFeedforward = feedforwardMin;
        return this;
    }

    /**
     * Sets the allowed error tolerance for the actuation group to be considered
     * "close enough" within the target position.
     *
     * @param tolerance
     * @return
     */
    public ActuatorGroupWrapper setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public void updateConstraints(ProfileConstraints constraints) {
        this.constraints = constraints;
//        setMotionProfile(new AsymmetricMotionProfile(position, targetPosition, constraints));
//        this.constr
    }

    public void updatePID(double P, double I, double D) {
        this.controller.setPID(P, I, D);
    }

    public void updateFeedforward(double ff) {
        this.currentFeedforward = ff;
    }

    /**
     * Gets the value read by the actuation group.
     *
     * @return double
     */
    public double getPosition() {
        return position;
    }

    /**
     * Gets the current target position for the actuation group.
     *
     * @return double
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getOverallTargetPosition() {
        return overallTargetPosition;
    }

    public double getPower() {
        return power;
    }

    public double getCurrentFeedforward() {
        return currentFeedforward;
    }

    /**
     * Gets any given HardwareDevice within this actuator group.
     *
     * @param deviceName The given HardwareMap name specified in the config,
     *                   or specified at object creation.
     * @return HardwareDevice object
     */
    public HardwareDevice getDevice(String deviceName) {
        return this.devices.get(deviceName);
    }

    /**
     * Returns a list of all given HardwareDevices.
     *
     * @return
     */
    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }

    public ProfileState getState() {
        return this.state;
    }

    public ProfileConstraints getConstraints() {
        return this.constraints;
    }

    /**
     * Returns whether or not the given actuation group is within error
     * tolerance of the final position.
     *
     * @return
     */
    public boolean hasReached() {
        return this.reached;
    }
}