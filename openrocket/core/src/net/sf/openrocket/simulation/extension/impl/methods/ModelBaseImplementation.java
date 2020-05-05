package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;

import java.util.*;
import java.util.function.Function;

public abstract class ModelBaseImplementation implements ModelInterface {
    OptimizedMap valueFunctionTable = null;
    public MDPDefinition definition;
    float stepDiscount = 0.9f;
    float terminalDiscount = 0.999f;
    float alpha = 0.1f;
    float exploration = 0.05f;

    public float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action)); }
    public float valueFunction(StateActionTuple stateActionTuple) {
        return valueFunctionTable.get(stateActionTuple);
    }

    public OptimizedMap getValueFunctionTable() {
        return this.valueFunctionTable;
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        this.valueFunctionTable = valueFunctionTable;
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

    public void updateStepCustomFunction(ArrayList<StateActionTuple> SA, Formula reward) {
        //System.out.println("Step Combined method");
        updateStepCommon(
                SA,
                reward::evaluateFloat
        );
    }
    public void updateTerminalCustomFunction(ArrayList<StateActionTuple> SA, Formula terminalReward, Formula reward) {
        updateTerminalCommon(
                SA,
                terminalReward::evaluateFloat,
                reward::evaluateFloat
        );
    }
}
