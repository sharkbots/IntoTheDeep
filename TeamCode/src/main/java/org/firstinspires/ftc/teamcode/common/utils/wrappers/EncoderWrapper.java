package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class EncoderWrapper implements HardwareDevice {
    public Motor.Encoder encoder;
    public EncoderDirection direction = EncoderDirection.FORWARD;

    public EncoderWrapper(Motor.Encoder encoder) {
        this.encoder = encoder;
    }

    public enum EncoderDirection{
        FORWARD(1),
        REVERSE(-1);
        final int v;

        EncoderDirection(int v) {
            this.v = v;
        }
        public int getMultiplier(){
            return v;
        }
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

    public void setDirection(EncoderDirection direction){
        this.direction = direction;
    }

    @Override
    public void close() {

    }

    public void reset() {encoder.reset();}

    public double getPosition() {
        return this.encoder.getPosition()*direction.getMultiplier();
    }

    public double getRawVelocity() {
        return this.encoder.getRawVelocity()*direction.getMultiplier();
    }

    public double getCorrectedVelocity() {
        return this.encoder.getCorrectedVelocity()*direction.getMultiplier();
    }
}