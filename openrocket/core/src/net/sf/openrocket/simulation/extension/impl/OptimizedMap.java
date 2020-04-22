package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;

import java.util.*;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

/**
    valueFunctionTable
        StateActionTuples --> float

    --- State ---
    0: altitude
    1: velocity
    2: thrust
    3: angleX
    4: angleZ
    5: gimbalX
    6: gimbalY

    --- Action ---
    7: thrust
    8: gimbalX
    9: gimbalY
 */

public class OptimizedMap {
    enum MapMethod {
        Traditional, Coupled
    }
    public static MapMethod mapMethod = MapMethod.Coupled;

    private float[] valueFunctionTable = null;
    private float[] landerValueFunctionTable = null;
    private float[] stabilizerValueFunctionTable = null;
    private float[] reacherValueFunctionTable = null;

    public OptimizedMap() {
        if (mapMethod == MapMethod.Traditional)
            constructorTraditional(null);
        else if (mapMethod == MapMethod.Coupled)
            constructorCoupled(null, null, null);
    }

    public OptimizedMap(float[] newValueFunctionTable) {
        constructorTraditional(newValueFunctionTable);
    }

    private void deepCopyElementsFromKnownHashMapToOtherMap(HashMap<String, HashMap> fromHashMap, HashMap<String, HashMap> destinationHashMap) {
        destinationHashMap.put("meta", deepCopyStringHashMap(fromHashMap.get("meta")));
        destinationHashMap.put("stateDefinition", deepCopyFloatArrayHashMap(fromHashMap.get("stateDefinition")));
        destinationHashMap.put("actionDefinition", deepCopyFloatArrayHashMap(fromHashMap.get("actionDefinition")));
        if (fromHashMap.containsKey("formulas"))
            destinationHashMap.put("formulas", deepCopyStringHashMap(fromHashMap.get("formulas")));
        if (fromHashMap.containsKey("noActionState"))
            destinationHashMap.put("noActionState", deepCopyFloatArrayHashMap(fromHashMap.get("noActionState")));
        destinationHashMap.put("successConditions", deepCopyFloatArrayHashMap(fromHashMap.get("successConditions")));
    }

    private HashMap<String, String> deepCopyStringHashMap(HashMap<String, String> origin) {
        HashMap<String, String> destination = new HashMap<>();
        for (Map.Entry<String, String> entry : origin.entrySet()) {
            String newStringKey = String.valueOf(entry.getKey());
            String newStringValue = String.valueOf(entry.getValue());
            destination.put(newStringKey, newStringValue);
        }
        return destination;
    }

    private HashMap<String, float[]> deepCopyFloatArrayHashMap(HashMap<String, float[]> origin) {
        HashMap<String, float[]> destination = new HashMap<>();
        for (Map.Entry<String, float[]> entry : origin.entrySet()) {
            String newStringKey = String.valueOf(entry.getKey());
            int size = entry.getValue().length;
            float[] duplicatedFloatArray = new float[size];
            for (int i = 0; i < size; i++) duplicatedFloatArray[i] = entry.getValue()[i] + 0.0f;
            destination.put(newStringKey, duplicatedFloatArray);
        }
        return destination;
    }

    private void constructorTraditional(float[] newValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newValueFunctionTable == null)
            newValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(generalDefinition));
        this.valueFunctionTable = newValueFunctionTable;
    }

    public OptimizedMap(float[] newLanderValueFunctionTable, float[] newReacherValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCoupled(newLanderValueFunctionTable, newReacherValueFunctionTable, newStabilizerValueFunctionTable);
    }

    private void constructorCoupled(float[] newLanderValueFunctionTable, float[] newReacherValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newLanderValueFunctionTable == null)
            newLanderValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(landerDefinition));
        this.landerValueFunctionTable = newLanderValueFunctionTable;
        checkTableValues(landerDefinition);
        if (newStabilizerValueFunctionTable == null)
            newStabilizerValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(stabilizerDefinition));
        this.stabilizerValueFunctionTable = newStabilizerValueFunctionTable;
        checkTableValues(stabilizerDefinition);
        if (newReacherValueFunctionTable == null)
            newReacherValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(reacherDefinition));
        this.reacherValueFunctionTable = newReacherValueFunctionTable;
        checkTableValues(reacherDefinition);
    }

    public void checkTableValues(HashMap<String, HashMap> definition) {
        String name = (String)definition.get("meta").get("name");
        float table[] = null;
        if (name.equals("general")) table = this.valueFunctionTable;
        else if (name.equals("lander")) table = this.landerValueFunctionTable;
        else if (name.equals("stabilizer")) table = this.stabilizerValueFunctionTable;
        else if (name.equals("reacher")) table = this.reacherValueFunctionTable;
        else System.out.println("TABLE TYPE DOES NOT EXIST!");

        int[] indeces = getIndecesFromDefinition(definition);
        int product = indexProduct(indeces);
        for (int i = 0; i < product; i++) {
            if (Float.isNaN(table[i])) {
                System.out.println("NAN IN TABLE!");
            }
        }
    }

    private void constructorCode() {
        deepCopyElementsFromKnownHashMapToOtherMap(_generalDefinition, generalDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_landerDefinition, landerDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_reacherDefinition, reacherDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_stabilizerDefinition, stabilizerDefinition);

        convertAnglesToRadians(generalDefinition);
        convertAnglesToRadians(landerDefinition);
        convertAnglesToRadians(reacherDefinition);
        convertAnglesToRadians(stabilizerDefinition);

        addIntegersToDefinition(generalDefinition);
        addIntegersToDefinition(landerDefinition);
        addIntegersToDefinition(reacherDefinition);
        addIntegersToDefinition(stabilizerDefinition);

        generateAndIndecesToDefinition(generalDefinition);
        generateAndIndecesToDefinition(landerDefinition);
        generateAndIndecesToDefinition(reacherDefinition);
        generateAndIndecesToDefinition(stabilizerDefinition);

        convertStringFormulasToFormulas(generalDefinition);
        convertStringFormulasToFormulas(landerDefinition);
        convertStringFormulasToFormulas(reacherDefinition);
        convertStringFormulasToFormulas(stabilizerDefinition);
    }

    public float get(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.get(this, stateActionTuple);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.put(this, stateActionTuple, newValue);
    }

    public static boolean needToChooseNewAction(State newState, State oldState, Action lastAction) {
        if (lastAction == null) return true;
        if (isNoActionState(newState)) return false;
        if (equivalentState(newState, oldState)) return false;
        return true;
    }

    public static boolean isNoActionState(State state) {
        if (state == null) return true;

        if (state.definition.containsKey("noActionState")) {
            for (Object entryObject : state.definition.get("noActionState").entrySet()) {
                Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
                String field = entry.getKey();
                for (int i = 0; i < entry.getValue().length; i++) {
                    float equivalentValue = entry.getValue()[i];
                    if (state.getDouble(field) == equivalentValue)
                        return true;
                }
            }
        }
        return false;
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        if (state_a.definition != state_b.definition) return false;

        boolean equivalent = true;
        for (Object objectField : state_a.definition.get("stateDefinition").keySet()) {
            String stateField = (String)objectField;
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }


    public boolean containsKey(State state, Action action) {
        return true;
        // return checkBounds(new StateActionTuple(state, action));
    }

    public boolean containsKey(StateActionTuple stateActionTuple) {
        return true;
        // return checkBounds(stateActionTuple);
    }

    public int getMinField(int[] minMax) {
        return minMax[0];
    }

    public int getMaxField(int[] minMax) {
        return minMax[1];
    }

    public TerminationBooleans getTerminationValidity(CoupledStates coupledStates) {
        if (coupledStates == null) return new TerminationBooleans(null);

        ArrayList<Boolean> successBooleans = new ArrayList<>();
        for (State state: coupledStates) {
            successBooleans.add(_getTerminationValidity(state));
        }

        return new TerminationBooleans(successBooleans);
    }

    private boolean _getTerminationValidity(State state) {
        for (Object entryObject : state.definition.get("successConditions").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
            String field = entry.getKey();
            double currentValue = state.getDouble(field);
            float[] minMax = entry.getValue();
            if (currentValue < minMax[0]) { return false; }
            if (currentValue > minMax[1]) { return false; }
        }
        return true;
    }

    private float[] allocateNewValueFunctionTable(int[] indeces) {
        System.out.println("Allocating stateSpace: " + indexProduct(indeces));
        return DynamicValueFunctionTable.callAllocation(indexProduct(indeces));
    }

    private void generateAndIndecesToDefinition(HashMap<String, HashMap> MDPDefinition){
        int[] indeces = new int[MDPDefinition.get("stateDefinitionIntegers").size() + MDPDefinition.get("actionDefinitionIntegers").size()];
        int index = 0;

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>)entryObject;
            indeces[index] = (getMaxField(entry.getValue()) - getMinField(entry.getValue()) + 1);
            index += 1;
        }
        for (Object entryObject : MDPDefinition.get("actionDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>)entryObject;
            indeces[index] = (getMaxField(entry.getValue()) - getMinField(entry.getValue()) + 1);
            index += 1;
        }
        MDPDefinition.put("indeces",
            new HashMap<String, int[]>() {{
                put("indeces", indeces);
        }});
    }

    public static void convertStringFormulasToFormulas(HashMap<String, HashMap> MDPDefinition){
        HashMap<String, Formula> formulaHashMap = new HashMap<>();
        if (!MDPDefinition.containsKey("formulas")) return;
        for (Object entryObject : MDPDefinition.get("formulas").entrySet()) {
            Map.Entry<String, String> entry = (Map.Entry<String, String>) entryObject;
            String assignToField = entry.getKey();
            try {
                String formula = entry.getValue();
            } catch (ClassCastException e) {
                // already ran this code and converted formulas
                // happens rarely but it means we already did the conversions
                return;
            }
            String formula = entry.getValue();
            formulaHashMap.put(assignToField, ExpressionEvaluator.getInstance().generateFormula(formula));
        }
        MDPDefinition.put("formulas", formulaHashMap);
    }

    public static int indexProduct(int[] indeces) {
        int totalSize = 1;
        for (int index: indeces) {
            totalSize *= index;
        }
        return totalSize;
    }

    public float[] getValueFunctionArray() {
        return valueFunctionTable;
    }

    public float[] getLanderValueFunctionArray() {
        return landerValueFunctionTable;
    }

    public float[] getStabilizerValueFunctionArray() {
        return stabilizerValueFunctionTable;
    }

    public float[] getReacherValueFunctionArray() {
        return reacherValueFunctionTable;
    }

    public static CoupledActions combineCoupledActions(Action landerAction, Action stabilizerAction) {
        return new CoupledActions(landerAction, stabilizerAction, stabilizerAction);
    }

    public static CoupledActions combineCoupledTripleActions(Action landerAction, Action gimbalXAction, Action gimbalYAction) {
        return new CoupledActions(landerAction, gimbalXAction, gimbalYAction);
    }

    private int[] getIndecesFromDefinition(HashMap<String, HashMap> MDPDefinition){
        return (int[])MDPDefinition.get("indeces").get("indeces");
    }

    private void convertAnglesToRadians(HashMap<String, HashMap> MDPDefinition) {
        _convertAnglesToRadians(MDPDefinition, "stateDefinition");
        _convertAnglesToRadians(MDPDefinition, "actionDefinition");
        _convertAnglesToRadians(MDPDefinition, "successConditions");
    }

    private void _convertAnglesToRadians(HashMap<String, HashMap> definition, String definitionField) {
        for (Object entryObject : definition.get(definitionField).entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>)entryObject;
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

    private void addIntegersToDefinition(HashMap<String, HashMap> MDPDefinition) {
        State state = new State(null);
        state.definition = MDPDefinition;

        // state section
        HashMap<String, int[]> integerFields = new HashMap<>();
        for (Object entryObject : MDPDefinition.get("stateDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>)entryObject;
            String field = entry.getKey();
            float[] fieldDefinition = entry.getValue();
            float minFloatValue = fieldDefinition[0];  // low
            int minIntValue = (int)state.setDouble(field, minFloatValue).get(field);
            float maxFloatValue = fieldDefinition[1];  // high
            int maxIntValue = (int)state.setDouble(field, maxFloatValue).get(field);
            integerFields.put(field, new int[]{minIntValue, maxIntValue});
        }
        MDPDefinition.put("stateDefinitionIntegers", integerFields);

        // action section
        integerFields = new HashMap<>();
        for (Object entryObject : MDPDefinition.get("actionDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>)entryObject;
            String field = entry.getKey();
            float[] fieldDefinition = entry.getValue();
            float minFloatValue = fieldDefinition[0];  // low
            int minIntValue = (int)state.setDouble(field, minFloatValue).get(field);
            float maxFloatValue = fieldDefinition[1];  // high
            int maxIntValue = (int)state.setDouble(field, maxFloatValue).get(field);
            integerFields.put(field, new int[]{minIntValue, maxIntValue});
        }
        MDPDefinition.put("actionDefinitionIntegers", integerFields);
    }
}
