package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

public abstract class ModelBaseImplementation implements ModelInterface {
    OptimizedMap valueFunctionTable = null;
    float stepDiscount = 0.9f;
    float terminalDiscount = 1f;
    float alpha = 0.3f;

    public float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action)); }
    public float valueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.get(stateActionTuple);
    }

    public float landingValueFunction(State state, Action action) { return landingValueFunction(new StateActionTuple(state, action)); }
    public float landingValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getLander(stateActionTuple);
    }
    public float stabilizingValueFunction(State state, Action action) { return stabilizingValueFunction(new StateActionTuple(state, action)); }
    public float stabilizingValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getStabilizer(stateActionTuple);
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

    public void updateStepCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward
    ) {}

    public void updateTerminalCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<StateActionTuple.State, Float> reward
    ) {}

    /**
     * Code below here should NOT BE MODIFIED.  IT allowed for the explicit format of the functions that is present.
     * The implementation is common to all methods, and allows for drastic code reuse.
     **/

    /** Combined (Traditional) Implementation **/

    public void updateStepFunction(ArrayList<StateActionTuple> stateActionTuples) {
        //System.out.println("Step Combined method");
        updateStepCommon(
            stateActionTuples,
            this::valueFunction,
            valueFunctionTable::put,
            this::reward
        );
    }
    public void updateTerminalFunction(ArrayList<StateActionTuple> stateActionTuples) {
        System.out.println("Terminal Combined method");
        updateTerminalCommon(
            stateActionTuples,
            this::terminalReward,
            this::valueFunction,
            valueFunctionTable::put,
            this::reward
        );
    }

    /** Landing method references **/

    public void updateStepLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        //System.out.println("Step Landing method");
        updateStepCommon(
            stateActionTuples,
            this::landingValueFunction,
            valueFunctionTable::putLander,
            this::rewardLanding
        );
    }

    public void updateTerminalLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        System.out.println("Terminal Landing method");
        updateTerminalCommon(
            stateActionTuples,
            this::terminalLandingReward,
            this::landingValueFunction,
            valueFunctionTable::putLander,
            this::rewardLanding
        );
    }

    /** Stabilizing method references **/

    public void updateStepStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        //System.out.println("Step Stabilizing method");
        updateStepCommon(
            stateActionTuples,
            this::stabilizingValueFunction,
            valueFunctionTable::putStabilizer,
            this::rewardStabilizing
        );
    }

    public void updateTerminalStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        System.out.println("Terminal Stabilizing method");
        updateTerminalCommon(
            stateActionTuples,
            this::terminalStabilizingReward,
            this::stabilizingValueFunction,
            valueFunctionTable::putStabilizer,
            this::rewardStabilizing
        );
    }
}
