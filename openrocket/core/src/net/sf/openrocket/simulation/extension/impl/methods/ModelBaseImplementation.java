package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;

public abstract class ModelBaseImplementation implements ModelInterface {
    OptimizedMap valueFunctionTable = null;
    public HashMap<String, HashMap> definition;
    float stepDiscount = 0.7f;
    float terminalDiscount = 0.999f;
    float alpha = 0.2f;

    public float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action, generalDefinition)); }
    public float valueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.get(stateActionTuple);
    }

    public float landerValueFunction(State state, Action action) { return landerValueFunction(new StateActionTuple(state, action, landerDefinition)); }
    public float landerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getLander(stateActionTuple);
    }
    public float stabilizerValueFunction(State state, Action action) { return stabilizerValueFunction(new StateActionTuple(state, action, stabilizerDefinition)); }
    public float stabilizerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getStabilizer(stateActionTuple);
    }
    public float reachingValueFunction(State state, Action action) { return reachingValueFunction(new StateActionTuple(state, action, reacherDefinition)); }
    public float reachingValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getReacher(stateActionTuple);
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
            ArrayList<StateActionTuple> SA,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward,
            BiFunction<State, State, Boolean> equivalentState
    ) {}

    public void updateTerminalCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<StateActionTuple.State, Float> reward,
            BiFunction<State, State, Boolean> equivalentState
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
            this::valueFunction,
            valueFunctionTable::put,
            this::reward,
            OptimizedMap::equivalentState
        );
    }
    public void updateTerminalFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Combined method");
        updateTerminalCommon(
            SA,
            this::terminalReward,
            this::valueFunction,
            valueFunctionTable::put,
            this::reward,
            OptimizedMap::equivalentState
        );
    }

    /** Lander method references **/

    public void updateStepLanderFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Lander method");
        updateStepCommon(
            SA,
            this::landerValueFunction,
            valueFunctionTable::putLander,
            this::rewardLander,
            OptimizedMap::equivalentStateLander
        );
    }

    public void updateTerminalLanderFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Lander method");
        updateTerminalCommon(
            SA,
            this::terminalLanderReward,
            this::landerValueFunction,
            valueFunctionTable::putLander,
            this::rewardLander,
            OptimizedMap::equivalentStateLander
        );
    }

    /** Stabilizer method references **/

    public void updateStepStabilizerFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Stabilizer method");
        updateStepCommon(
            SA,
            this::stabilizerValueFunction,
            valueFunctionTable::putStabilizer,
            this::rewardStabilizer,
            OptimizedMap::equivalentStateStabilizer
        );
    }

    public void updateTerminalStabilizerFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Stabilizer method");
        updateTerminalCommon(
            SA,
            this::terminalStabilizerReward,
            this::stabilizerValueFunction,
            valueFunctionTable::putStabilizer,
            this::rewardStabilizer,
            OptimizedMap::equivalentStateStabilizer
        );
    }

    /** Reaching method references **/

    public void updateStepReachingFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Stabilizer method");
        updateStepCommon(
                SA,
                this::reachingValueFunction,
                valueFunctionTable::putReacher,
                this::rewardReaching,
                OptimizedMap::equivalentStateReacher
        );
    }

    public void updateTerminalReachingFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Stabilizer method");
        updateTerminalCommon(
                SA,
                this::terminalReachingReward,
                this::reachingValueFunction,
                valueFunctionTable::putReacher,
                this::rewardReaching,
                OptimizedMap::equivalentStateReacher
        );
    }

    // general ----- NOTE: PRECISIONS ARE NOT CORRECT!
    public static HashMap<String, HashMap> generalDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("altitude", new float[]{0, 1, 0.25f});
            put("positionX", new float[]{0, 1, 0.25f});
            put("positionY", new float[]{0, 1, 0.25f});
            put("velocity", new float[]{0, 1, 0.25f});
            put("time", new float[]{0, 1, 0.25f});
            put("angleX", new float[]{0, 1, 0.25f});
            put("angleZ", new float[]{0, 1, 0.25f});
            put("thrust", new float[]{0, 1, 0.25f});
            put("gimbleY", new float[]{0, 1, 0.25f});
            put("gimbleZ", new float[]{0, 1, 0.25f});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("thrust", new float[]{-6, 6, 2});
            put("gimbleY", new float[]{-6, 6, 2});
            put("gimbleZ", new float[]{-6, 6, 2});
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "general");
        }});
    }};


    // lander ----- NOTE: PRECISIONS ARE NOT CORRECT!
    public static HashMap<String, HashMap> landerDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("position", new float[]{0, 8, 2});
            put("altitude", new float[]{0, 1, 0.25f});
            put("velocity", new float[]{-15, 15, 5});
            put("time", new float[]{-4, 4, 2});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
        }});
        put("formulas", new HashMap<String, String>() {{
            put("position", "add(abs(positionX),abs(positionY))");
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "lander");
        }});
    }};

    // reacher
    public static HashMap<String, HashMap> reacherDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
            put("angle", new float[]{-15, 15, 5});
            put("position", new float[]{-4, 4, 2});
            put("velocity", new float[]{-3, 3, 1});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("gimbal", new float[]{-6, 6, 2});
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "reacher");
            put("simmetrical", "true");
        }});
    }};

    // stabilizer
    public static HashMap<String, HashMap> stabilizerDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("time", new float[]{0, 5, 1});
            put("thrust", new float[]{0, 1, 0.25f});
            put("angle", new float[]{-12, 12, 2});
            put("angleVelocity", new float[]{-6, 6, 2});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 0.5f});
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "stabilizer");
            put("simmetrical", "true");
        }});
    }};

}
