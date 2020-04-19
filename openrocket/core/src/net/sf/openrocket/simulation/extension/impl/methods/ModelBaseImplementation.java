package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;

public abstract class ModelBaseImplementation implements ModelInterface {
    OptimizedMap valueFunctionTable = null;
    public HashMap<String, HashMap> definition;
    float stepDiscount = 0.9f;
    float terminalDiscount = 0.999f;
    float alpha = 0.1f;

    public float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action)); }
    public float valueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
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

    /** Lander method references **/

    public void updateStepLanderFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Lander method");
        updateStepCommon(
            SA,
            this::rewardLander
        );
    }

    public void updateTerminalLanderFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Lander method");
        updateTerminalCommon(
            SA,
            this::terminalLanderReward,
            this::rewardLander
        );
    }

    /** Stabilizer method references **/

    public void updateStepStabilizerFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Stabilizer method");
        updateStepCommon(
            SA,
            this::rewardStabilizer
        );
    }

    public void updateTerminalStabilizerFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Stabilizer method");
        updateTerminalCommon(
            SA,
            this::terminalStabilizerReward,
            this::rewardStabilizer
        );
    }

    /** Reaching method references **/

    public void updateStepReachingFunction(ArrayList<StateActionTuple> SA) {
        //System.out.println("Step Stabilizer method");
        updateStepCommon(
                SA,
                this::rewardReaching
        );
    }

    public void updateTerminalReachingFunction(ArrayList<StateActionTuple> SA) {
        // System.out.println("Terminal Stabilizer method");
        updateTerminalCommon(
                SA,
                this::terminalReachingReward,
                this::rewardReaching
        );
    }

    // Model base implementation is the schema of the MDP
    // angles are in degrees and later converted to radians ONLY IF angle >= 0.1

    public static void printDefinition(HashMap<String, HashMap> definition) {
        System.out.println("definition={");
        System.out.println("    meta=" + ((HashMap<String, String>)definition.get("meta")).toString());
        System.out.println("    stateDefinition=" + ((HashMap<String, float[]>)definition.get("stateDefinition")).toString());
        System.out.println("    actionDefinition=" + ((HashMap<String, float[]>)definition.get("actionDefinition")).toString());
        System.out.println("    stateDefinitionIntegers=" + ((HashMap<String, float[]>)definition.get("stateDefinitionIntegers")).toString());
        System.out.println("    actionDefinitionIntegers=" + ((HashMap<String, float[]>)definition.get("actionDefinitionIntegers")).toString());
        System.out.println("    indeces=" + ((int[])definition.get("indeces").get("indeces")).toString());
        if (definition.containsKey("formulas"))
            System.out.println("    formulas=" + ((HashMap<String, Formula>)definition.get("formulas")).toString());
        System.out.println("}");
    }

    // general ----- NOTE: PRECISIONS ARE NOT CORRECT!
    public static HashMap<String, HashMap> generalDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("positionX", new float[]{0, 1, 0.25f});
            put("positionY", new float[]{0, 1, 0.25f});
            put("positionZ", new float[]{0, 1, 0.25f});
            put("velocityZ", new float[]{0, 1, 0.25f});
            put("time", new float[]{0, 1, 0.25f});
            put("angleX", new float[]{0, 1, 0.25f});
            put("angleZ", new float[]{0, 1, 0.25f});
            put("thrust", new float[]{0, 1, 0.25f});
            put("gimbalX", new float[]{0, 1, 0.25f});
            put("gimbalY", new float[]{0, 1, 0.25f});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("thrust", new float[]{-6, 6, 2});
            put("gimbalX", new float[]{-6, 6, 2});
            put("gimbalY", new float[]{-6, 6, 2});
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "general");
        }});
    }};



    public static HashMap<String, HashMap> landerDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("angle", new float[]{-35, 35, 5});
            // put("position", new float[]{0, 8, 2});

            // put("positionZ", new float[]{0, 50, 1});  // 2.5
            put("log2PositionZ", new float[]{0, 5.6f, 0.3f});
            // put("log8PositionZ", new float[]{0, 1.9f, 0.05f});
            // put("velocityZ", new float[]{-30, 5, 2.5f});  // 5
            put("log2VelocityZ", new float[]{-5, 2.3f, 0.3f});
            // put("log8VelocityZ", new float[]{-1.6f, 0.8f, 0.05f});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
        }});
        put("formulas", new HashMap<String, String>() {{
            put("position", "Add(Abs(positionX),Abs(positionY))");
            put("angle", "Add(Abs(angleX),Abs(angleY))");
            put("log2PositionZ", "Log2(Add(positionZ,1))");
            put("log8PositionZ", "Log8(Add(positionZ,1))");
            put("log2VelocityZ", "Mult(Signum(velocityZ),Log2(Add(Abs(velocityZ),1)))");
            put("log8VelocityZ", "Mult(Signum(velocityZ),Log8(Add(Abs(velocityZ),1)))");

            // put("MDPDecision", "And(Gt(Abs(positionX), 4.0), Le(Abs(angleX), Asin(Div(PI,8))))");
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
            put("symmetry", "angle,position,velocity,gimbal");
        }});
    }};

    // stabilizer
    public static HashMap<String, HashMap> stabilizerDefinition = new HashMap<String, HashMap>() {{
        put("stateDefinition",  new HashMap<String, float[]>() {{
            put("time", new float[]{0, 5, 1});
            put("thrust", new float[]{0, 1, 0.25f});
            put("angle", new float[]{-12, 12, 2});
            put("angleVelocity", new float[]{-12, 12, 4});
        }});
        put("actionDefinition", new HashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 0.5f});
        }});
        put("meta", new HashMap<String, String>() {{
            put("name", "stabilizer");
            put("symmetry", "angle,angleVelocity,gimbal");
        }});
    }};

}
