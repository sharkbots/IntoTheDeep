package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class EncoderWrapper implements HardwareDevice {
    public Motor.Encoder encoder;

    public EncoderWrapper(Motor.Encoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return encoder.toString();
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

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public double getRawVelocity() {
        return this.encoder.getRawVelocity();
    }

    public double getCorrectedVelocity() {
        return this.encoder.getCorrectedVelocity();
    }
}