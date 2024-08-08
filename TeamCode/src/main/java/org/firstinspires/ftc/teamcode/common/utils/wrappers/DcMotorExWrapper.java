package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorExWrapper implements DcMotorEx {

    private final DcMotorEx dcMotor;
    double overridePower = 0;

    public DcMotorExWrapper(DcMotorEx motor) {
        this.dcMotor = motor;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return dcMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        overridePower = power;
        dcMotor.setPower(power);
    }

    public void setOverridePower(double power) {
        // Keep previous power value in overridePower
        dcMotor.setPower(power);
    }

    public void cancelOverridePower() {
        dcMotor.setPower(overridePower);
    }

    @Override
    public double getPower() {
        return dcMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dcMotor.close();
    }

    @Override
    public void setMotorEnable() {
        dcMotor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        dcMotor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return dcMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double v) {
        dcMotor.setVelocity(v);
    }

    @Override
    public void setVelocity(double v, AngleUnit angleUnit) {

    }

    @Override
    public double getVelocity() {
        return dcMotor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit angleUnit) {
        return dcMotor.getVelocity(angleUnit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode runMode, PIDCoefficients pidCoefficients) {
        dcMotor.setPIDCoefficients(runMode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode runMode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        dcMotor.setPIDFCoefficients(runMode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double v, double v1, double v2, double v3) {
        dcMotor.setVelocityPIDFCoefficients(v, v1, v2, v3);
    }

    @Override
    public void setPositionPIDFCoefficients(double v) {
        dcMotor.setPositionPIDFCoefficients(v);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode runMode) {
        return dcMotor.getPIDCoefficients(runMode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode runMode) {
        return dcMotor.getPIDFCoefficients(runMode);
    }

    @Override
    public void setTargetPositionTolerance(int i) {
        dcMotor.setTargetPositionTolerance(i);
    }

    @Override
    public int getTargetPositionTolerance() {
        return dcMotor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit currentUnit) {
        return dcMotor.getCurrent(currentUnit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit currentUnit) {
        return dcMotor.getCurrentAlert(currentUnit);
    }

    @Override
    public void setCurrentAlert(double v, CurrentUnit currentUnit) {
        dcMotor.setCurrentAlert(v, currentUnit);
    }

    @Override
    public boolean isOverCurrent() {
        return dcMotor.isOverCurrent();
    }
}
