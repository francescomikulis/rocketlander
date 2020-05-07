package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.*;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;

import java.io.Serializable;
import java.util.*;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class MDPDefinition implements Serializable {
    private static Gson gson = new GsonBuilder().setPrettyPrinting().create();

    public String name;
    public String methodName;
    public int priority = 1;
    public transient ModelBaseImplementation[] models = null;
    public String reward;
    transient Formula _reward;
    public String terminalReward;
    transient Formula _terminalReward;
    public double discount = 0.999;
    public double stepDiscount = 0.9;
    public double alpha = 0.1;
    public double exploration = 0.05;
    public String[] symmetryAxes = null;
    public boolean passDownSymmetryAxis = false;
    public transient HashSet<String> symmetryAxesHashSet = null;
    public LinkedHashMap<String, float[]> stateDefinition = null;
    public transient String[] stateDefinitionFields = null;
    public LinkedHashMap<String, float[]> actionDefinition = null;
    public transient String[] actionDefinitionFields = null;
    public LinkedHashMap<String, ArrayList<String>> MDPSelectionFormulas = null;
    transient LinkedHashMap<String, ArrayList<Object[]>> _MDPSelectionFormulas = null;
    public LinkedHashMap<String, String[]> childrenMDPOptions = null;
    public LinkedHashMap<String, String> formulas = null;
    transient LinkedHashMap<String, Formula> _formulas = null;
    public transient LinkedHashMap<String, LinkedHashMap<String, String>> symmetryFormulas = null;
    public LinkedHashMap<String, float[]> noActionState = null;
    public LinkedHashMap<String, float[]> successConditions = null;

    public transient ValueFunction valueFunction = null;

    public transient int indexProduct = 0;
    public transient int temporaryIndexProductForAction = 0;
    public transient int[] indeces = null;
    public transient LinkedHashMap<String, Integer> reverseIndeces = null;
    public transient int[][] stateDefinitionIntegers = null;
    public transient int[][] actionDefinitionIntegers = null;
    public transient LinkedHashMap<String, Integer> childrenMDPIntegerOptions = null;
    public transient LinkedHashMap<String, Float> precisions = null;
    public transient LinkedHashMap<String, Float> rangeShifts = null;

    public transient boolean tryToReadFromFile = true;


    public MDPDefinition() {
        super();
    }

    public void postConstructor() {
        setReward(reward);
        setTerminalReward(terminalReward);
        setMDPSelectionFormulas(MDPSelectionFormulas);
        setFormulas(formulas);
        generateSymmetryFormulas();

        setupRLMethod();
        generateIterationFields();

        setAllTransientProperties();
    }

    public static MDPDefinition buildFromJsonString(String jsonString) {
        MDPDefinition definition = gson.fromJson(jsonString, MDPDefinition.class);
        definition.postConstructor();
        return definition;
    }

    public static String toJsonString(MDPDefinition definition) {
        return gson.toJson(definition);
    }

    public void setValueFunction(ValueFunction valueFunction) {
        this.valueFunction = valueFunction;
    }


    /** Business logic **/

    public static boolean needToChooseNewAction(State newState, State oldState, Action lastAction) {
        if (lastAction == null) return true;
        if (isNoActionState(newState)) return false;
        if (newState.definition.stateDefinition.size() == 0) return true;
        if (equivalentState(newState, oldState)) return false;
        return true;
    }

    public static boolean isNoActionState(State state) {
        if (state == null) return true;
        if ((state.definition.noActionState == null) || (state.definition.noActionState.size() == 0)) return false;

        for (Map.Entry<String, float[]> entry: state.definition.noActionState.entrySet()) {
            String field = entry.getKey();
            for (int i = 0; i < entry.getValue().length; i++) {
                float equivalentValue = entry.getValue()[i];
                if (state.getDouble(field) == equivalentValue)
                    return true;
            }
        }
        return false;
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        if (state_a.definition != state_b.definition) return false;

        boolean equivalent = true;
        for (Object objectField : state_a.definition.stateDefinitionFields) {
            String stateField = (String)objectField;
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }


    public static TerminationBooleans getTerminationValidity(CoupledStates coupledStates) {
        if (coupledStates == null)
            return new TerminationBooleans(null);

        ArrayList<Boolean> successBooleans = new ArrayList<>();
        for (State state: coupledStates) {
            successBooleans.add(_getTerminationValidity(state));
        }
        return new TerminationBooleans(successBooleans);
    }

    private static boolean _getTerminationValidity(State state) {
        for (Map.Entry<String, float[]> entry: state.definition.successConditions.entrySet()) {
            String field = entry.getKey();
            double currentValue = state.getDouble(field);
            float[] minMax = entry.getValue();
            if (currentValue < minMax[0]) { return false; }
            if (currentValue > minMax[1]) { return false; }
        }
        return true;
    }


    /** Other constructor set and getter formulas **/

    public void setReward(String rewardString) {
        this.reward = rewardString;
        this._reward = ExpressionEvaluator.getInstance().generateFormula(this.reward);
    }

    public void setTerminalReward(String terminalRewardString) {
        this.terminalReward = terminalRewardString;
        if (this.terminalReward == null) {
            this._terminalReward = null;
            return;
        }
        this._terminalReward = ExpressionEvaluator.getInstance().generateFormula(this.terminalReward);
    }

    public void setFormulas(LinkedHashMap<String, String> formulas) {
        this.formulas = formulas;
        if (this.formulas == null) {
            this._formulas = null;
            return;
        }
        this._formulas = new LinkedHashMap<>();
        for (Map.Entry<String, String> entry: this.formulas.entrySet()) {
            this._formulas.put(entry.getKey(), ExpressionEvaluator.getInstance().generateFormula(entry.getValue()));
        }
    }

    public void setMDPSelectionFormulas(LinkedHashMap<String, ArrayList<String>> MDPSelectionFormulas) {
        this.MDPSelectionFormulas = MDPSelectionFormulas;
        if (this.MDPSelectionFormulas == null) {
            this._MDPSelectionFormulas = null;
            return;
        }
        this._MDPSelectionFormulas = new LinkedHashMap<>();
        for (Map.Entry<String, ArrayList<String>> entry: this.MDPSelectionFormulas.entrySet()) {
            ArrayList<String> options = entry.getValue();
            ArrayList<Object[]> conditionResults = new ArrayList<>();
            for (int i=0; i<options.size(); i+=2) {
                Object[] conditionResult = new Object[2];
                conditionResult[0] = ExpressionEvaluator.getInstance().generateFormula(options.get(i));  // formula
                conditionResult[1] = options.get(i + 1); // result
                conditionResults.add(conditionResult);
            }
            this._MDPSelectionFormulas.put(entry.getKey(), conditionResults);
        }
    }

    public void generateSymmetryFormulas() {
        if ((symmetryAxes == null) || (symmetryAxes.length == 0)) return;
        symmetryAxesHashSet = new HashSet<>(Arrays.asList(symmetryAxes));

        symmetryFormulas = new LinkedHashMap<>();
        for (String axis: new String[]{"X", "Y"}) {
            symmetryFormulas.put(axis, new LinkedHashMap<>());
            for (String symmetryAxis: symmetryAxes) {
                symmetryFormulas.get(axis).put(symmetryAxis, symmetryAxis + axis);
            }
        }
    }

    public void setupRLMethod() {
        String methods[] = methodName.replaceAll(" ", "").split(",");
        models = new ModelBaseImplementation[methods.length];
        for (int i = 0; i < models.length; i++) {
            models[i] = setupRLMethod(methods[i]);
        }
    }

    public ModelBaseImplementation setupRLMethod(String methodName) {
        ModelBaseImplementation model;
        switch (methodName.toUpperCase()) {
            case "MC":
                model = new MonteCarlo(this); break;
            case "TD0":
                model = new TD0(this); break;
            default:
                System.out.println("METHOD NAME NOT DEFINED IN THE IMPLEMENTATION.  Must choose between MC, TD0, SARSA.");
                return null;
        }
        model.setTerminalDiscount((float)discount);
        model.setStepDiscount((float)stepDiscount);
        model.setAlpha((float)alpha);
        model.setExploration((float)exploration);
        return model;
    }

    public void generateIterationFields() {
        stateDefinitionFields = new String[stateDefinition.size()];
        int index = 0;
        for (String stateField: stateDefinition.keySet()) {
            stateDefinitionFields[index] = stateField;
            index++;
        }

        actionDefinitionFields = new String[actionDefinition.size()];
        index = 0;
        for (String stateField: actionDefinition.keySet()) {
            actionDefinitionFields[index] = stateField;
            index++;
        }
    }

    /** Index computation code **/

    public static int computeIndex(StateActionTuple stateActionTuple) {
        return computeIndexState(stateActionTuple.state) + computeIndexAction(stateActionTuple.action);
    }

    public static int computeIndexState(State state) {
        int index = 0;
        int currentSize = 0;
        MDPDefinition definition = state.definition;
        int indeces[] = definition.indeces;
        int product = definition.indexProduct;

        int i = 0;
        for (String stateField: definition.stateDefinitionFields) {
            int[] minMax = definition.stateDefinitionIntegers[i];
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) state.get(stateField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
            i++;
        }
        return index;
    }

    public static int computeIndexAction(Action action) {
        int index = 0;
        MDPDefinition definition = action.definition;
        int currentSize = definition.stateDefinitionFields.length;
        int indeces[] = definition.indeces;
        int product = definition.temporaryIndexProductForAction;

        int i = 0;
        for (String actionField: definition.actionDefinitionFields) {
            int[] minMax = definition.actionDefinitionIntegers[i];
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) action.get(actionField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
            i++;
        }
        return index;
    }


    /** Set all transient properties **/

    private void setAllTransientProperties() {
        convertAnglesToRadians();
        buildChildrenMDPIntegerOptions();
        predefineAllPrecisions();
        generateStateActionDefinitionIntegers();
        generateIndecesDefinition();
    }

    private void convertAnglesToRadians() {
        _convertAnglesToRadians(stateDefinition);
        _convertAnglesToRadians(actionDefinition);
        _convertAnglesToRadians(successConditions);
    }

    private void _convertAnglesToRadians(LinkedHashMap<String, float[]> definitionField) {
        for (Map.Entry<String, float[]> entry : definitionField.entrySet()) {
            String field = entry.getKey();
            if (field.toLowerCase().contains("angle") || field.toLowerCase().contains("gimbal")) {
                boolean convert = false;
                int size = entry.getValue().length;
                if (size == 3) convert = entry.getValue()[2] >= 0.1;
                else if (size == 2) convert = Math.abs(entry.getValue()[0]) >= 2 || Math.abs(entry.getValue()[1]) >= 2;
                if (convert) {
                    float[] definitionValues = entry.getValue();
                    for (int i = 0; i < size; i++)
                        definitionValues[i] *= (Math.PI / 180);
                }
            }
        }
    }


    private void buildChildrenMDPIntegerOptions() {
        if ((childrenMDPOptions == null) || (childrenMDPOptions.size() == 0)) return;

        LinkedHashMap<String, Integer> MDPIntegerHashMap = new LinkedHashMap<>();
        for (Map.Entry<String, String[]> entry: childrenMDPOptions.entrySet()) {
            for (String MDPName: entry.getValue()) {
                if (MDPIntegerHashMap.containsKey(MDPName)) continue;
                MDPIntegerHashMap.put(MDPName, MDPIntegerHashMap.size());
            }
        }
        childrenMDPIntegerOptions = MDPIntegerHashMap;

        for (String field: actionDefinitionFields) {
            float[] def = actionDefinition.get(field);
            if (field.contains("MDP")) {
                int minValue = MDPIntegerHashMap.size() - 1;
                int maxValue = 0;
                for (String MDPName: childrenMDPOptions.get(field)) {
                    minValue = Math.min(minValue, MDPIntegerHashMap.get(MDPName));
                    maxValue = Math.max(maxValue, MDPIntegerHashMap.get(MDPName));
                }
                def[0] = minValue;
                def[1] = maxValue;
                def[2] = 1;
            }
        }
    }

    public void predefineAllPrecisions() {
        precisions = new LinkedHashMap<>();
        rangeShifts = new LinkedHashMap<>();
        for (String stateField: stateDefinitionFields) {
            float precision = getPrecisionFromField(stateField);
            precisions.put(stateField, precision);
            float rangeShift = getRangeShiftFromField(stateField);
            rangeShifts.put(stateField, rangeShift);
            if ((symmetryAxesHashSet != null) && symmetryAxesHashSet.contains(stateField)) {
                // maybe bad idea to do this
                precisions.put(stateField + "X", precision);
                precisions.put(stateField + "Y", precision);
                precisions.put(stateField + "Z", precision);
                rangeShifts.put(stateField + "X", rangeShift);
                rangeShifts.put(stateField + "Y", rangeShift);
                rangeShifts.put(stateField + "Z", rangeShift);
            }
        }
        for (String actionField: actionDefinitionFields) {
            float precision = getPrecisionFromField(actionField);
            precisions.put(actionField, precision);
            float rangeShift = getRangeShiftFromField(actionField);
            rangeShifts.put(actionField, rangeShift);
            if ((symmetryAxesHashSet != null) && symmetryAxesHashSet.contains(actionField)) {
                // maybe bad idea to do this
                precisions.put(actionField + "X", precision);
                precisions.put(actionField + "Y", precision);
                precisions.put(actionField + "Z", precision);
                rangeShifts.put(actionField + "X", rangeShift);
                rangeShifts.put(actionField + "Y", rangeShift);
                rangeShifts.put(actionField + "Z", rangeShift);
            }
        }
    }

    public float getPrecisionFromField(String field) {
        if (stateDefinition.containsKey(field)) {
            return stateDefinition.get(field)[2];
        } else if (actionDefinition.containsKey(field)) {
            return actionDefinition.get(field)[2];
        }
        if (field.contains("X")) {
            return getPrecisionFromField(field.replace("X", ""));
        } else if (field.contains("Y")) {
            return getPrecisionFromField(field.replace("Y", ""));
        }
        return 0.0000001f;
    }

    private boolean _closeEnoughToBeConsideredZero(float value, float precision) {
        float remainder = Math.abs(value - (int)value);
        float threshold = 0.1f * precision;  // within a tenth of the precision
        return remainder < threshold;
    }

    private float _getRangeShiftFromField(float[] definition) {
        // how much needs to be subtracted for the first positive value to be zero
        float currentValue = definition[0];
        while (currentValue < 0) {
            if (_closeEnoughToBeConsideredZero(currentValue, definition[2]))
                return 0;
            currentValue += definition[2];
        }
        return currentValue;
    }

    public float getRangeShiftFromField(String field) {
        if (stateDefinition.containsKey(field)) {
            return _getRangeShiftFromField(stateDefinition.get(field));
        } else if (actionDefinition.containsKey(field)) {
            return _getRangeShiftFromField(actionDefinition.get(field));
        }
        if (field.contains("X")) {
            return getRangeShiftFromField(field.replace("X", ""));
        } else if (field.contains("Y")) {
            return getRangeShiftFromField(field.replace("Y", ""));
        }
        return 0;
    }

    private void generateStateActionDefinitionIntegers() {
        StateActionTuple.State state = new StateActionTuple.State(null);
        state.definition = this;

        reverseIndeces = new LinkedHashMap<>();

        stateDefinitionIntegers = new int[stateDefinition.size()][];
        int i = 0;
        for (String field: stateDefinitionFields) {
            reverseIndeces.put(field, i);

            float[] fieldDefinition = stateDefinition.get(field);
            float minFloatValue = fieldDefinition[0];  // low
            int minIntValue = (int)state.setDouble(field, minFloatValue).get(field);
            float maxFloatValue = fieldDefinition[1];  // high
            int maxIntValue = (int)state.setDouble(field, maxFloatValue).get(field);
            stateDefinitionIntegers[i] = new int[]{minIntValue, maxIntValue};
            i++;
        }

        StateActionTuple.Action action = new StateActionTuple.Action();
        action.definition = this;

        actionDefinitionIntegers = new int[actionDefinition.size()][];
        i = 0;
        for (String field: actionDefinitionFields) {
            reverseIndeces.put(field, i + stateDefinitionFields.length);

            float[] fieldDefinition = actionDefinition.get(field);
            float minFloatValue = fieldDefinition[0];  // low
            int minIntValue = (int)action.setDouble(field, minFloatValue).get(field);
            float maxFloatValue = fieldDefinition[1];  // high
            int maxIntValue = (int)action.setDouble(field, maxFloatValue).get(field);
            actionDefinitionIntegers[i] = new int[]{minIntValue, maxIntValue};
            i++;
        }
    }


    private void generateIndecesDefinition(){
        int[] indeces = new int[stateDefinitionIntegers.length + actionDefinitionIntegers.length];
        int index = 0;
        int indexProduct = 1;

        for (int[] minMax: stateDefinitionIntegers) {
            indeces[index] = (minMax[1] - minMax[0] + 1);
            indexProduct *= indeces[index];
            index += 1;
        }

        int temporaryIndexProductForAction = 1;
        for (int[] minMax: actionDefinitionIntegers) {
            indeces[index] = (minMax[1] - minMax[0] + 1);
            indexProduct *= indeces[index];
            temporaryIndexProductForAction *= indeces[index];
            index += 1;
        }
        this.temporaryIndexProductForAction = temporaryIndexProductForAction;

        this.indeces = indeces;
        this.indexProduct = indexProduct;
    }
}
