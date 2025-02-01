package org.firstinspires.ftc.teamcode.common.utils.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.pedropathing.pathgen.MathFunctions;

import java.lang.Math;
import java.util.Set;

@Config
public class AbsoluteAnalogEncoder implements HardwareDevice {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;
    private boolean wraparound;

    private String name;
    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    public AbsoluteAnalogEncoder zero(double offset){
        this.offset = offset;
        return this;
    }


    public AbsoluteAnalogEncoder setInverted(boolean invert){
      inverted = invert;
      return this;
    }

    public AbsoluteAnalogEncoder setWraparound(boolean wraparound) {
        this.wraparound = wraparound;
        return this;
    }

    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;
    public double getCurrentPosition() {
        double pos = MathFunctions.normalizeAngle((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        if (wraparound && pos > Math.PI * 1.5) {
            pos -= Math.PI * 2;
        }
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(MathFunctions.normalizeAngle(pastPosition)) > 0.1 || Math.abs(MathFunctions.normalizeAngle(pos)) < 1) pastPosition = pos;
        return pastPosition;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
    @Override
    public String getDeviceName() {
        return "AbsoluteAnalogEncoder";
    }

    public void resolveName(HardwareMap hwmap){
        this.name = fetchDeviceName(hwmap, this.encoder);
    }

    private static String fetchDeviceName(HardwareMap hardwareMap, AnalogInput analogInput) {
        Set<String> names = hardwareMap.getNamesOf(analogInput);
        return names.isEmpty() ? "Unknown" : names.iterator().next();
    }

    public String getConfigName(){
        return name;
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
}