package org.firstinspires.ftc.teamcode.common.utils.Menu;


// This is what an action is: a function with no input parameters which returns
// true/false when evaluate.
interface ActionFunction {
    Boolean evaluate();
}

public class Action {
    private final ActionFunction performAction; // A function that performs the action and returns true if the action is complete
    private final String msg;

    // Constructor that sets the action
    public Action(String msg, ActionFunction performAction) {
        this.performAction = performAction;
        this.msg = msg;
    }

    // Method to determine if the action is complete based on the BooleanSupplier
    public boolean evaluate() {
        if (msg != null){
            // Global.telemetry.addLine(msg);
        }
        return performAction.evaluate();
    }
}