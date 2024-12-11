package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Set;


public class ServoWrapper implements Servo {

    private ServoImplEx servo;
    private String name;
    private double offset = 0.0;


    public ServoWrapper(ServoImplEx servo) {
        this.servo = servo;
        this.name = "Unknown";
    }

    public void resolveName(HardwareMap hardwareMap) {
        this.name = fetchDeviceName(hardwareMap, this.servo);
    }

    private static String fetchDeviceName(HardwareMap hardwareMap, Servo servo) {
        Set<String> names = hardwareMap.getNamesOf(servo);
        return names.isEmpty() ? "Unknown" : names.iterator().next();
    }

    public String getConfigName() {return name;}

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return this.offset;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        this.servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return this.servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        this.servo.setPosition(position - offset);
    }

    @Override
    public double getPosition() {
        return this.servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        this.servo.scaleRange(min, max);
    }

    public void setPwmDisable(){
        this.servo.setPwmDisable();
    }

    public void setPwmEnable(){
        this.servo.setPwmEnable();
    }
}