package org.firstinspires.ftc.teamcode.common.utils.Menu;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.utils.wrappers.SubsystemWrapper;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;


public class ConfigMenu extends SubsystemWrapper {

    private final Robot robot;
    GamepadEx operator;
    Object object;
    Object fieldBackup = null;

    Field[] fields;
    int currentField = 0;

    StateMachine sm;
    StateMachine.State navigateMenu, editMenuItem, lockMenu;

    Telemetry telemetry;


    public ConfigMenu(GamepadEx operator_, Telemetry telemetry_) {
        operator = operator_;
        telemetry = telemetry_;

        robot = Robot.getInstance();

        sm = new StateMachine();
        navigateMenu = new StateMachine.State("navigateMenu");
        editMenuItem = new StateMachine.State("editMenuItem");
        lockMenu = new StateMachine.State("lockMenu");
        sm.setInitialState(navigateMenu);

        navigateMenu.addTransitionTo(
                editMenuItem,
                () -> operator.wasJustPressed(GamepadKeys.Button.A),
                new Actions(new Action("enterEdit", this::backupCurrentField)));

        editMenuItem.addTransitionTo(
                navigateMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.A),
                new Actions(new Action("nextMenuItem", () -> { fieldBackup = null; return true; })));

        editMenuItem.addTransitionTo(
                navigateMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.B),
                new Actions( new Action("nextMenuItem", this::restoreCurrentField)));

        navigateMenu.addTransitionTo(
                navigateMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                new Actions(new Action("nextMenuItem", this::nextMenuItem)));

        navigateMenu.addTransitionTo(
                navigateMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                new Actions(new Action("previousMenuItem", this::previousMenuItem)));

        editMenuItem.addTransitionTo(
                editMenuItem,
                () -> operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                new Actions(new Action("incrementField", () -> changeFieldBy(1.0))));

        editMenuItem.addTransitionTo(
                editMenuItem,
                () -> operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                new Actions(new Action("decrementField", () -> changeFieldBy(-1.0))));

        editMenuItem.addTransitionTo(
                editMenuItem,
                () -> operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                new Actions(new Action("incrementField", () -> changeFieldBy(0.1))));

        editMenuItem.addTransitionTo(
                editMenuItem,
                () -> operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                new Actions(new Action("decrementField", () -> changeFieldBy(-0.1))));

        navigateMenu.addTransitionTo(
                lockMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.X),
                new Actions(new Action("menu locked", () -> true)));

        lockMenu.addTransitionTo(
                navigateMenu,
                () -> operator.wasJustPressed(GamepadKeys.Button.X),
                new Actions(new Action("menu locked", () -> true)));
    }


    public void setConfigurationObject(Object object){
        this.object = object;
        this.fields = object.getClass().getDeclaredFields();
    }


    public Boolean backupCurrentField() {
        Field field = fields[currentField];
        field.setAccessible(true);
        try {
            fieldBackup = field.get(object);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return true;
    }


    private Boolean restoreCurrentField() {
        Field field = fields[currentField];
        field.setAccessible(true);
        try {
            field.set(object, fieldBackup);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return true;
    }

    private boolean changeFieldBy(double value)  {
        assert(value!=0);
        Field field = fields[currentField];
        field.setAccessible(true);
        Class<?> type = field.getType();
        telemetry.addLine(type.getName());
        int intValue = (int)Math.round(value); // used of integer like values below after unboxing field
        if(intValue==0) { // case where double input is within ]-0.5;+0.5[
            intValue = value>=0? 1:-1;
        }

        try {
            if (type.equals(Integer.class) || type.equals(int.class)) {
                field.setInt(object, field.getInt(object) + intValue);
            } else if (type.equals(Double.class) || type.equals(double.class)) {
                field.setDouble(object, field.getDouble(object) + value);
            } else if (type.equals(Float.class) || type.equals(float.class)) {
                field.setFloat(object, field.getFloat(object) + (float)value);
            } else if (type.equals(Boolean.class) || type.equals(boolean.class)) {
                Boolean currentValue = (Boolean) field.get(object);
                field.set(object, !currentValue); // Toggle the boolean value
            } else if (type.isEnum()) {
                Object[] enumValues = type.getEnumConstants();
                Object currentValue = field.get(object);
                for (int i = 0; i < enumValues.length; i++) {
                    if (enumValues[i].equals(currentValue)) {
                        int len = enumValues.length;
                        Object nextValue = enumValues[(((i + intValue) % len) + len) % len];//Java modulo conserves sign: -1%5 = -1 and not 4, need positive index all the time
                        field.set(object, nextValue);
                        break;
                    }
                }
            }
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        return true;
    }

    private boolean previousMenuItem() {
        currentField = Range.clip(currentField-1, 0, fields.length-1);
        return true;
    }

    public boolean isLocked(){
        if (sm.currentState!=lockMenu){
            return false;
        }
        else {
            return true;
        }
    }
    private boolean nextMenuItem() {
        currentField = Range.clip(currentField+1, 0, fields.length-1);
        return true;
    }

    public void updateDisplay() {
        // Assuming Global.telemetry is accessible and correct.
        try {
            if(sm.currentState==lockMenu){
                telemetry.addLine(bold(color("Menu values are locked.", "red")));
                currentField = 0;
            }
            // Display static fields
            for (int i = 0; i < fields.length; i++) {
                Field field = fields[i];
                field.setAccessible(true);
                Object value = field.get(Modifier.isStatic(field.getModifiers()) ? null : object); // Use null for static fields
                telemetry.addData(sm.currentState==lockMenu? unformattedFieldName(i) : formattedFieldName(i),
                        value != null ? (sm.currentState==lockMenu? value.toString():formattedValue(i, value.toString())) : "null");
            }

        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        telemetry.update();
    }

    private String formattedFieldName(int fieldIndex) {
        return formattedValue(fieldIndex, fields[fieldIndex].getName());
    }
    private String unformattedFieldName(int fieldIndex) {
        return fields[fieldIndex].getName();
    }

    private String formattedValue(int fieldIndex, String value) {
        if (fieldIndex==currentField) {
            value = bold(value);
            value = color(value, (sm.currentState==navigateMenu)?"cyan":"yellow");
        }
        return value;
    }

    private String bold(String string) {
        return "<b>"+string+"</b>";
    }
    private String italic(String string) {
        return "<i>"+string+"</i>";
    }
    private String color(String string, String color) {
        return "<font color = \""+color+"\">"+string+"</font>";
    }

    @Override
    public void periodic() {
        operator.readButtons();
        sm.updateState();
        updateDisplay();
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }
}
