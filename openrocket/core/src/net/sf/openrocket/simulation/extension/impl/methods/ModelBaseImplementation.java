package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;

import java.util.*;
import java.util.function.BiFunction;
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

    public void removeValueFunctionTable() {
        this.valueFunctionTable = null;
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
        if (definition.containsKey("symmetryFormulasX"))
            System.out.println("    symmetryFormulasX=" + ((HashMap<String, Formula>)definition.get("symmetryFormulasX")).toString());
        if (definition.containsKey("symmetryFormulasY"))
            System.out.println("    symmetryFormulasY=" + ((HashMap<String, Formula>)definition.get("symmetryFormulasY")).toString());
        if (definition.containsKey("formulas"))
            System.out.println("    formulas=" + ((HashMap<String, Formula>)definition.get("formulas")).toString());
        if (definition.containsKey("noActionState"))
            System.out.println("    noActionState=" + ((HashMap<String, float[]>)definition.get("noActionState")).toString());
        if (definition.containsKey("successConditions"))
            System.out.println("    successConditions=" + ((HashMap<String, float[]>)definition.get("successConditions")).toString());
        System.out.println("}");
    }


    public static MDPDefinition getLanderDefinition() {
        MDPDefinition landerDefinition = new MDPDefinition();
        landerDefinition.name = "lander";
        landerDefinition.methodName = "MC";
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


    public static MDPDefinition getReacherDefinition() {
        MDPDefinition reacherDefinition = new MDPDefinition();
        reacherDefinition.name = "reacher";
        reacherDefinition.methodName = "TD0";
        reacherDefinition.reward = "Add(-Mult(position, 10.0), -velocity)";
        reacherDefinition.symmetryAxes = new String[]{"angle","position","velocity","gimbal"};
        reacherDefinition.stateDefinition = new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
            put("log2Angle", new float[]{-2, 2, 0.5f});
            put("position", new float[]{-9, 9, 3});
            put("velocity", new float[]{-3, 3, 1});
        }};
        reacherDefinition.actionDefinition = new LinkedHashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 1});
        }};
        reacherDefinition.noActionState = new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0.0f});
        }};
        reacherDefinition.formulas = new LinkedHashMap<String, String>() {{
            put("log2Angle", "Mult(Signum(angle),Log2(Add(Abs(angle),1)))");
            put("log2Position", "Mult(Signum(position),Log2(Add(Abs(position),1)))");
        }};
        reacherDefinition.successConditions = new LinkedHashMap<String, float[]>() {{
            put("position", new float[]{-10, 10});
            put("angle", new float[]{-12, 12});
        }};
        reacherDefinition.postConstructor();

        String str = MDPDefinition.toJsonString(reacherDefinition);
        MDPDefinition def = MDPDefinition.buildFromJsonString(str);

        return reacherDefinition;
    }

    public static MDPDefinition getStabilizerDefinition() {
        MDPDefinition stabilizerDefinition = new MDPDefinition();
        stabilizerDefinition.name = "stabilizer";
        stabilizerDefinition.methodName = "TD0";
        stabilizerDefinition.reward = "Add(-Pow(Todeg(angle), 2.0), 1.0)";
        stabilizerDefinition.symmetryAxes = new String[]{"angle","angleVelocity","gimbal"};
        stabilizerDefinition.stateDefinition = new LinkedHashMap<String, float[]>() {{
            put("time", new float[]{0, 5, 1});
            put("thrust", new float[]{0, 1, 0.25f});
            put("log2Angle", new float[]{-2.195f, 2.195f, 0.3659f});
            put("angleVelocity", new float[]{-12, 12, 4});
        }};
        stabilizerDefinition.actionDefinition = new LinkedHashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 1f});
        }};
        stabilizerDefinition.noActionState = new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0.0f});
        }};
        stabilizerDefinition.formulas = new LinkedHashMap<String, String>() {{
            put("log2Angle", "Mult(Signum(angle),Log2(Add(Abs(angle),1)))");
        }};
        stabilizerDefinition.successConditions = new LinkedHashMap<String, float[]>() {{
            put("angle", new float[]{-8, 8});
        }};
        stabilizerDefinition.postConstructor();

        String str = MDPDefinition.toJsonString(stabilizerDefinition);
        MDPDefinition def = MDPDefinition.buildFromJsonString(str);

        return stabilizerDefinition;
    }




    // general ----- NOTE: PRECISIONS ARE NOT CORRECT!
    public static HashMap<String, LinkedHashMap> generalDefinition = new HashMap<String, LinkedHashMap>();
    public static HashMap<String, LinkedHashMap> _generalDefinition = new HashMap<String, LinkedHashMap>() {{
        put("meta", new LinkedHashMap<String, String>() {{
            put("name", "general");
        }});
        put("stateDefinition",  new LinkedHashMap<String, float[]>() {{
            //put("positionX", new float[]{0, 1, 0.25f});
            //put("positionY", new float[]{0, 1, 0.25f});
            put("log2PositionZ", new float[]{0, 5.6f, 0.6f});
            put("log2VelocityZ", new float[]{-5, 2.3f, 0.6f});
            // put("time", new float[]{0, 5, 1});
            put("log2AngleX", new float[]{-2, 2, 0.5f});
            put("log2AngleY", new float[]{-2, 2, 0.5f});
        }});
        put("actionDefinition", new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
            put("gimbalX", new float[]{-3, 3, 1f});
            put("gimbalY", new float[]{-3, 3, 1f});
        }});
        put("formulas", new LinkedHashMap<String, String>() {{
            put("log2AngleX", "Mult(Signum(angleX),Log2(Add(Abs(angleX),1)))");
            put("log2AngleY", "Mult(Signum(angleY),Log2(Add(Abs(angleY),1)))");
            put("log2PositionZ", "Log2(Add(positionZ,1))");
            put("log2VelocityZ", "Mult(Signum(velocityZ),Log2(Add(Abs(velocityZ),1)))");
        }});
        put("successConditions", new LinkedHashMap<String, float[]>() {{
            put("velocityZ", new float[]{-2, 2});
            // put("position", new float[]{-4, 4});
            put("angle", new float[]{-8, 8});
        }});
    }};


    public static HashMap<String, LinkedHashMap> landerDefinition = new HashMap<>();
    public static HashMap<String, LinkedHashMap> _landerDefinition = new HashMap<String, LinkedHashMap>() {{
        put("meta", new LinkedHashMap<String, String>() {{
            put("name", "lander");
            put("methodName", "MC");
            put("reward", "-Div(Abs(thrust),100.0)");
            put("terminalReward", "-Abs(velocityZ)");

            /*
            put("discount", "0.999");
            put("stepDiscount", "0.9");
            put("alpha", "0.1");
            put("exploration", "0.05");
             */
        }});
        put("stateDefinition",  new LinkedHashMap<String, float[]>() {{
            put("angle", new float[]{-35, 35, 5});
            // put("position", new float[]{0, 8, 2});

            // put("positionZ", new float[]{0, 50, 1});  // 2.5
            put("log2PositionZ", new float[]{0, 5.6f, 0.3f});
            // put("log8PositionZ", new float[]{0, 1.9f, 0.05f});
            // put("velocityZ", new float[]{-30, 5, 2.5f});  // 5
            put("log2VelocityZ", new float[]{-5, 2.3f, 0.3f});
            // put("log8VelocityZ", new float[]{-1.6f, 0.8f, 0.05f});
        }});
        put("actionDefinition", new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
            //put("gimbalMDPX", new float[]{0, 0, 0});
            //put("gimbalMDPY", new float[]{0, 0, 0});
        }});
        // selection formulas override the traditionally available list of choices
        put("MDPSelectionFormulas", new LinkedHashMap<String, ArrayList<String>>() {{
            // format is like a swith case statement - last case is the default even if false!
            put("gimbalMDPX", new ArrayList<>(Arrays.asList(
                    "And(Gt(Abs(positionX),3.0), FALSE)", "reacher",
                    "And(Le(Abs(positionX),3.0), Le(Abs(angleX),Asin(Div(PI,16))))", "stabilizer"
            )));
            put("gimbalMDPY", new ArrayList<>(Arrays.asList(
                    "And(Gt(Abs(positionY),3.0), FALSE)", "reacher",
                    "And(Le(Abs(positionY),3.0), Le(Abs(angleY),Asin(Div(PI,16))))", "stabilizer"
            )));
        }});
        put("childrenMDPOptions", new LinkedHashMap<String, String>() {{
            put("gimbalMDPX", "stabilizer,reacher");
            put("gimbalMDPY", "stabilizer,reacher");
        }});
        put("formulas", new LinkedHashMap<String, String>() {{
            put("position", "Add(Abs(positionX),Abs(positionY))");
            put("angle", "Add(Abs(angleX),Abs(angleY))");
            put("log2PositionZ", "Log2(Add(positionZ,1))");
            put("log8PositionZ", "Log8(Add(positionZ,1))");
            put("log2VelocityZ", "Mult(Signum(velocityZ),Log2(Add(Abs(velocityZ),1)))");
            put("log8VelocityZ", "Mult(Signum(velocityZ),Log8(Add(Abs(velocityZ),1)))");

            // put("MDPDecision", "And(Gt(Abs(positionX), 4.0), Le(Abs(angleX), Asin(Div(PI,8))))");
        }});
        put("successConditions", new LinkedHashMap<String, float[]>() {{
            put("velocityZ", new float[]{-2, 2});
        }});
    }};

    // reacher
    public static HashMap<String, LinkedHashMap> reacherDefinition = new HashMap<>();
    public static HashMap<String, LinkedHashMap> _reacherDefinition = new HashMap<String, LinkedHashMap>() {{
        put("meta", new LinkedHashMap<String, String>() {{
            put("name", "reacher");
            put("methodName", "TD0");
            put("reward", "Add(-Mult(position, 10.0), -velocity)");
            put("symmetry", "angle,position,velocity,gimbal");
        }});
        put("stateDefinition",  new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0, 1, 0.25f});
            // put("angle", new float[]{-15, 15, 5});
            put("log2Angle", new float[]{-2, 2, 0.5f});
            put("position", new float[]{-9, 9, 3});
            // put("log2Position", new float[]{-4, 4, 1f});
            put("velocity", new float[]{-3, 3, 1});
        }});
        put("actionDefinition", new LinkedHashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 1});
        }});
        put("noActionState", new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0.0f});
        }});
        put("formulas", new LinkedHashMap<String, String>() {{
            put("log2Angle", "Mult(Signum(angle),Log2(Add(Abs(angle),1)))");
            put("log2Position", "Mult(Signum(position),Log2(Add(Abs(position),1)))");
        }});
        put("successConditions", new LinkedHashMap<String, float[]>() {{
            put("position", new float[]{-10, 10});
            put("angle", new float[]{-12, 12});
        }});
    }};

    // stabilizer
    public static HashMap<String, LinkedHashMap> stabilizerDefinition = new LinkedHashMap<>();
    public static HashMap<String, LinkedHashMap> _stabilizerDefinition = new LinkedHashMap<String, LinkedHashMap>() {{
        put("meta", new LinkedHashMap<String, String>() {{
            put("name", "stabilizer");
            put("methodName", "TD0");
            put("reward", "Add(-Pow(Todeg(angle), 2.0), 1.0)");
            put("symmetry", "angle,angleVelocity,gimbal");
        }});
        put("stateDefinition",  new LinkedHashMap<String, float[]>() {{
            put("time", new float[]{0, 5, 1});
            put("thrust", new float[]{0, 1, 0.25f});
            // put("angle", new float[]{-12, 12, 2});
            put("log2Angle", new float[]{-2.195f, 2.195f, 0.3659f});
            put("angleVelocity", new float[]{-12, 12, 4});
        }});
        put("actionDefinition", new LinkedHashMap<String, float[]>() {{
            put("gimbal", new float[]{-3, 3, 1f});
        }});
        put("noActionState", new LinkedHashMap<String, float[]>() {{
            put("thrust", new float[]{0.0f});
        }});
        put("formulas", new LinkedHashMap<String, String>() {{
            put("log2Angle", "Mult(Signum(angle),Log2(Add(Abs(angle),1)))");
        }});
        put("successConditions", new LinkedHashMap<String, float[]>() {{
            put("angle", new float[]{-8, 8});
            // put("angleVelocity", new float[]{-8, 8});
        }});
    }};

}
