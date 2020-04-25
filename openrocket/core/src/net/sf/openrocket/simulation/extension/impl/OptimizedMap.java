package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

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

    public HashMap<String, float[]> valueFunctionTables = new HashMap<>();

    public OptimizedMap(LinkedHashMap<String, ModelBaseImplementation> methods) {
        constructorCode(methods, false);
    }

    public OptimizedMap(LinkedHashMap<String, ModelBaseImplementation> methods, boolean reset) {
        constructorCode(methods, reset);
    }

    private void setupDefinition(HashMap<String, LinkedHashMap> definition) {
        convertAnglesToRadians(definition);
        buildChildrenMDPIntegerOptions(definition);
        addIntegersToDefinition(definition);
        generateAndIndecesToDefinition(definition);
        convertStringFormulasToFormulas(definition);
    }

    private void constructorCode(LinkedHashMap<String, ModelBaseImplementation> methods, boolean reset) {
        deepCopyElementsFromKnownHashMapToOtherMap(_generalDefinition, generalDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_landerDefinition, landerDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_reacherDefinition, reacherDefinition);
        deepCopyElementsFromKnownHashMapToOtherMap(_stabilizerDefinition, stabilizerDefinition);

        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            HashMap<String, LinkedHashMap> definition = entry.getValue().definition;

            setupDefinition(definition);

            float[] valueFunctionTable = null;
            if (!reset) {
                valueFunctionTable = (float[]) definition.get("valueFunction").get("valueFunction");
            }
            if (valueFunctionTable == null) {
                valueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(definition));
                definition.get("valueFunction").put("valueFunction", valueFunctionTable);
            }
            valueFunctionTables.put(entry.getKey(), valueFunctionTable);
            checkTableValues(definition);
        }
    }

    private float[] allocateNewValueFunctionTable(int[] indeces) {
        int size = indexProduct(indeces);
        System.out.println("Allocating stateSpace: " + size);
        return new float[size];
    }

    public void checkTableValues(HashMap<String, LinkedHashMap> definition) {
        String name = (String)definition.get("meta").get("name");
        float[] table = valueFunctionTables.get(name);

        int[] indeces = getIndecesFromDefinition(definition);
        int product = indexProduct(indeces);
        for (int i = 0; i < product; i++) {
            if (Float.isNaN(table[i])) {
                System.out.println("NAN IN TABLE!");
            }
        }
    }

    public float get(StateActionTuple stateActionTuple) {
        String definitionName = (String)stateActionTuple.state.definition.get("meta").get("name");
        float[] valueFunctionTable = valueFunctionTables.get(definitionName);
        int index = computeIndex(stateActionTuple);
        return valueFunctionTable[index];
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        String definitionName = (String)stateActionTuple.state.definition.get("meta").get("name");
        float[] valueFunctionTable = valueFunctionTables.get(definitionName);
        int index = computeIndex(stateActionTuple);
        valueFunctionTable[index] = newValue;
        return newValue;
    }

    private static int computeIndex(StateActionTuple stateActionTuple) {
        return computeIndexState(stateActionTuple.state) + computeIndexAction(stateActionTuple.action);
    }

    public static int computeIndexState(State state) {
        int index = 0;
        int currentSize = 0;
        HashMap<String, LinkedHashMap> MDPDefinition = state.definition;
        int indeces[] = ((int[])(MDPDefinition.get("indeces").get("indeces")));
        int product = OptimizedMap.indexProduct(indeces);

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String stateField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) state.get(stateField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
        }
        return index;
    }

    public static int computeIndexAction(Action action) {
        int index = 0;
        int currentSize = 0;
        HashMap<String, LinkedHashMap> MDPDefinition = action.definition;
        int indeces[] = ((int[])(MDPDefinition.get("indeces").get("indeces")));
        int product = OptimizedMap.indexProduct(indeces);

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            product /= indeces[currentSize];
            currentSize += 1;
        }

        for (Object entryObject : MDPDefinition.get("actionDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String actionField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) action.get(actionField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
        }
        return index;
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

    private void buildChildrenMDPIntegerOptions(HashMap<String, LinkedHashMap> MDPDefinition) {
        if (!MDPDefinition.containsKey("childrenMDPOptions")) return;

        LinkedHashMap<String, Integer> MDPIntegerHashMap = new LinkedHashMap<>();
        for (Object entryObject : MDPDefinition.get("childrenMDPOptions").entrySet()) {
            Map.Entry<String, String> entry = (Map.Entry<String, String>) entryObject;
            String key = entry.getKey();
            String[] MDPNames = ((String) entry.getValue()).split(",");
            for (String MDPName: MDPNames) {
                if (MDPIntegerHashMap.containsKey(MDPName)) continue;
                MDPIntegerHashMap.put(MDPName, MDPIntegerHashMap.size());
            }
        }
        MDPDefinition.put("childrenMDPIntegerOptions", MDPIntegerHashMap);

        for (Object entryObject : MDPDefinition.get("actionDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
            String key = entry.getKey();
            if (key.contains("MDP")) {
                LinkedHashMap<String, String> childrenMDPOptions = MDPDefinition.get("childrenMDPOptions");
                String[] MDPNames = ((String) childrenMDPOptions.get(key)).split(",");
                int minValue = MDPIntegerHashMap.size() - 1;
                int maxValue = 0;
                for (String MDPName: MDPNames) {
                    minValue = Math.min(minValue, MDPIntegerHashMap.get(MDPName));
                    maxValue = Math.max(maxValue, MDPIntegerHashMap.get(MDPName));
                }
                entry.getValue()[0] = minValue;
                entry.getValue()[1] = maxValue;
                entry.getValue()[2] = 1;
            }
        }
    }

    private void generateAndIndecesToDefinition(HashMap<String, LinkedHashMap> MDPDefinition){
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
            new LinkedHashMap<String, int[]>() {{
                put("indeces", indeces);
        }});
    }

    public static void convertStringFormulasToFormulas(HashMap<String, LinkedHashMap> MDPDefinition){
        LinkedHashMap<String, Formula> formulaHashMap = new LinkedHashMap<>();
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

    private int[] getIndecesFromDefinition(HashMap<String, LinkedHashMap> MDPDefinition){
        return (int[])MDPDefinition.get("indeces").get("indeces");
    }

    private void convertAnglesToRadians(HashMap<String, LinkedHashMap> MDPDefinition) {
        _convertAnglesToRadians(MDPDefinition, "stateDefinition");
        _convertAnglesToRadians(MDPDefinition, "actionDefinition");
        _convertAnglesToRadians(MDPDefinition, "successConditions");
    }

    private void _convertAnglesToRadians(HashMap<String, LinkedHashMap> definition, String definitionField) {
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

    private void addIntegersToDefinition(HashMap<String, LinkedHashMap> MDPDefinition) {
        State state = new State(null);
        state.definition = MDPDefinition;

        // state section
        LinkedHashMap<String, int[]> integerFields = new LinkedHashMap<>();
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
        integerFields = new LinkedHashMap<>();
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


    /** DeepCopy Definition Code - This must be updated if the schema for definitions changes **/

    private void deepCopyElementsFromKnownHashMapToOtherMap(HashMap<String, LinkedHashMap> fromHashMap, HashMap<String, LinkedHashMap> destinationHashMap) {
        destinationHashMap.put("meta", deepCopyStringHashMap(fromHashMap.get("meta")));
        destinationHashMap.put("stateDefinition", deepCopyFloatArrayHashMap(fromHashMap.get("stateDefinition")));
        destinationHashMap.put("actionDefinition", deepCopyFloatArrayHashMap(fromHashMap.get("actionDefinition")));
        if (fromHashMap.containsKey("MDPSelectionFormulas"))
            destinationHashMap.put("MDPSelectionFormulas", deepCopyStringArrayListHashMap(fromHashMap.get("MDPSelectionFormulas")));
        if (fromHashMap.containsKey("childrenMDPOptions"))
            destinationHashMap.put("childrenMDPOptions", deepCopyStringHashMap(fromHashMap.get("childrenMDPOptions")));
        if (fromHashMap.containsKey("formulas"))
            destinationHashMap.put("formulas", deepCopyStringHashMap(fromHashMap.get("formulas")));
        if (fromHashMap.containsKey("noActionState"))
            destinationHashMap.put("noActionState", deepCopyFloatArrayHashMap(fromHashMap.get("noActionState")));
        destinationHashMap.put("successConditions", deepCopyFloatArrayHashMap(fromHashMap.get("successConditions")));
    }

    private LinkedHashMap<String, String> deepCopyStringHashMap(HashMap<String, String> origin) {
        LinkedHashMap<String, String> destination = new LinkedHashMap<>();
        for (Map.Entry<String, String> entry : origin.entrySet()) {
            String newStringKey = String.valueOf(entry.getKey());
            String newStringValue = String.valueOf(entry.getValue());
            destination.put(newStringKey, newStringValue);
        }
        return destination;
    }

    private LinkedHashMap<String, float[]> deepCopyFloatArrayHashMap(HashMap<String, float[]> origin) {
        LinkedHashMap<String, float[]> destination = new LinkedHashMap<>();
        for (Map.Entry<String, float[]> entry : origin.entrySet()) {
            String newStringKey = String.valueOf(entry.getKey());
            int size = entry.getValue().length;
            float[] duplicatedFloatArray = new float[size];
            for (int i = 0; i < size; i++) duplicatedFloatArray[i] = entry.getValue()[i] + 0.0f;
            destination.put(newStringKey, duplicatedFloatArray);
        }
        return destination;
    }

    private LinkedHashMap<String, ArrayList<String>> deepCopyStringArrayListHashMap(LinkedHashMap<String, ArrayList<String>> origin) {
        LinkedHashMap<String, ArrayList<String>> destination = new LinkedHashMap<>();
        for (Map.Entry<String, ArrayList<String>> entry : origin.entrySet()) {
            String newStringKey = String.valueOf(entry.getKey());
            ArrayList<String> newArrayList = new ArrayList<>();
            for (String s: entry.getValue()) {
                newArrayList.add(String.valueOf(s));
            }
            destination.put(newStringKey, newArrayList);
        }
        return destination;
    }
}
