package net.sf.openrocket.simulation.extension.impl.rocketlander.methods;

import net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.rocketlander.ValueFunctionManager;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions.Expression;

import java.util.*;
import java.util.function.Function;

public abstract class BaseMethodImplementation implements MethodInterface {
    ValueFunctionManager valueFunctionManager = null;
    public MDPDefinition definition;
    float stepDiscount = 0.9f;
    float terminalDiscount = 0.999f;
    float alpha = 0.1f;
    float exploration = 0.05f;

    public float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action)); }
    public float valueFunction(StateActionTuple stateActionTuple) {
        return valueFunctionManager.get(stateActionTuple);
    }

    public ValueFunctionManager getValueFunctionManager() {
        return this.valueFunctionManager;
    }

    public void setValueFunctionManager(ValueFunctionManager valueFunctionManager) {
        this.valueFunctionManager = valueFunctionManager;
    }

    public void setStepDiscount(float stepDiscount) { this.stepDiscount = stepDiscount; }
    public void setTerminalDiscount(float terminalDiscount) { this.terminalDiscount = terminalDiscount; }
    public void setAlpha(float alpha) { this.alpha = alpha; }
    public void setExploration(float exploration) { this.exploration = exploration; }
    public float getExploration() { return exploration; }

    public void updateStepCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> reward
    ) {}

    public void updateTerminalCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> terminalReward,
            Function<StateActionTuple.State, Float> reward
    ) {}

    /**
     * Code below here should NOT BE MODIFIED.  IT allowed for the explicit format of the functions that is present.
     * The implementation is common to all methods, and allows for drastic code reuse.
     **/

    /** Combined (Traditional) Implementation **/

    public void updateStepFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Combined method");
        updateStepCommon(
            SA,
            this::reward
        );
    }
    public void updateTerminalFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Combined method");
        updateTerminalCommon(
            SA,
            this::terminalReward,
            this::reward
        );
    }

    /** Dynamic Implementation **/

    public void updateStepCustomFunction(ArrayList<StateActionTuple> SA, Expression reward) {
        //System.out.println("Step Combined method");
        updateStepCommon(
                SA,
                reward::evaluateFloat
        );
    }
    public void updateTerminalCustomFunction(ArrayList<StateActionTuple> SA, Expression terminalReward, Expression reward) {
        updateTerminalCommon(
                SA,
                terminalReward::evaluateFloat,
                reward::evaluateFloat
        );
    }
}
