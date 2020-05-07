package net.sf.openrocket.simulation.extension.impl.rocketlander.methods;

import net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.rocketlander.ValueFunctionManager;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.CustomExpressionEvaluator.Formula;

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

    /** Default MDPDefinition ***/

    public static MDPDefinition getDefaultLanderDefinition() {
        MDPDefinition landerDefinition = new MDPDefinition();
        landerDefinition.name = "defaultLander";
        landerDefinition.methodName = "MC";
        landerDefinition.priority = 1;
        landerDefinition.reward = "-Div(Abs(thrust),100.0)";
        landerDefinition.terminalReward = "-Abs(velocityZ)";
        landerDefinition.stateDefinition = new LinkedHashMap<String, float[]>() {{
            put("angle", new float[]{-35, 35, 5});
            put("log2PositionZ", new float[]{0, 5.6f, 0.3f});
            put("log2VelocityZ", new float[]{-5, 2.3f, 0.3f});
        }};
        landerDefinition.actionDefinition = new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
        }};
        landerDefinition.MDPSelectionFormulas = new LinkedHashMap<String, ArrayList<String>>() {{
            put("gimbalMDPX", new ArrayList<>(Arrays.asList(
                    "And(Gt(Abs(positionX),3.0), FALSE)", "reacher",
                    "And(Le(Abs(positionX),3.0), Le(Abs(angleX),Asin(Div(PI,16))))", "stabilizer"
            )));
            put("gimbalMDPY", new ArrayList<>(Arrays.asList(
                    "And(Gt(Abs(positionY),3.0), FALSE)", "reacher",
                    "And(Le(Abs(positionY),3.0), Le(Abs(angleY),Asin(Div(PI,16))))", "stabilizer"
            )));
        }};
        landerDefinition.childrenMDPOptions = new LinkedHashMap<String, String[]>() {{
            put("gimbalMDPX", new String[]{"stabilizer","reacher"});
            put("gimbalMDPY", new String[]{"stabilizer","reacher"});
        }};

        landerDefinition.formulas = new LinkedHashMap<String, String>() {{
            put("position", "Add(Abs(positionX),Abs(positionY))");
            put("angle", "Add(Abs(angleX),Abs(angleY))");
            put("log2PositionZ", "Log2(Add(positionZ,1))");
            put("log8PositionZ", "Log8(Add(positionZ,1))");
            put("log2VelocityZ", "Mult(Signum(velocityZ),Log2(Add(Abs(velocityZ),1)))");
            put("log8VelocityZ", "Mult(Signum(velocityZ),Log8(Add(Abs(velocityZ),1)))");
        }};
        landerDefinition.successConditions = new LinkedHashMap<String, float[]>() {{
            put("velocityZ", new float[]{-2, 2});
        }};
        landerDefinition.postConstructor();

        String str = MDPDefinition.toJsonString(landerDefinition);
        MDPDefinition def = MDPDefinition.buildFromJsonString(str);

        return landerDefinition;
    }
}
