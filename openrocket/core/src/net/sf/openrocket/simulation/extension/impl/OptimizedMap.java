package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.*;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import static net.sf.openrocket.simulation.extension.impl.DynamicValueFunctionTable.*;
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
        if (newStabilizerValueFunctionTable == null)
            newStabilizerValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(stabilizerDefinition));
        this.stabilizerValueFunctionTable = newStabilizerValueFunctionTable;
        if (newReacherValueFunctionTable == null)
            newReacherValueFunctionTable = allocateNewValueFunctionTable(getIndecesFromDefinition(reacherDefinition));
        this.reacherValueFunctionTable = newReacherValueFunctionTable;
    }

    private void constructorCode() {
        addIntegersToDefinition(generalDefinition);
        addIntegersToDefinition(landerDefinition);
        addIntegersToDefinition(reacherDefinition);
        addIntegersToDefinition(stabilizerDefinition);

        generateAndIndecesToDefinition(generalDefinition);
        generateAndIndecesToDefinition(landerDefinition);
        generateAndIndecesToDefinition(reacherDefinition);
        generateAndIndecesToDefinition(stabilizerDefinition);
    }

    public float get(StateActionTuple stateActionTuple) {
        int[] indeces = getIndecesFromDefinition(stateActionTuple.state.definition);
        return DynamicValueFunctionTable.get(this, indeces, stateActionTuple);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        int[] indeces = getIndecesFromDefinition(stateActionTuple.state.definition);
        return DynamicValueFunctionTable.put(this, indeces, stateActionTuple, newValue);
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
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

    public TerminationBooleanTuple alterTerminalStateIfFailure(State state) {
        boolean verticalSuccess = true;
        boolean angleSuccess = true;

        if (state == null) return new TerminationBooleanTuple(true, true);
        for (Object entryObject : landerDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>)entryObject;
            String landerField = entry.getKey();
            int minValue = getMinField(entry.getValue());
            int maxValue = getMaxField(entry.getValue());
            int currentValue = (int)state.get(landerField);
            if (currentValue < minValue) { verticalSuccess = false; state.set(landerField, minValue); }
            if (currentValue > maxValue) { verticalSuccess = false; state.set(landerField, maxValue); }
        }
        for (Object entryObject : stabilizerDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>)entryObject;
            String stabilizerField = entry.getKey();
            int minValue = getMinField(entry.getValue());
            int maxValue = getMaxField(entry.getValue());
            int currentValue = (int)state.get(stabilizerField);
            if (currentValue < minValue) { angleSuccess = false; state.set(stabilizerField, minValue); }
            if (currentValue > maxValue) { angleSuccess = false; state.set(stabilizerField, maxValue); }
        }
        for (Object entryObject : reacherDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>)entryObject;
            String reacherField = entry.getKey();
            int minValue = getMinField(entry.getValue());
            int maxValue = getMaxField(entry.getValue());
            int currentValue = (int)state.get(reacherField);
            if (currentValue < minValue) { angleSuccess = false; state.set(reacherField, minValue); }
            if (currentValue > maxValue) { angleSuccess = false; state.set(reacherField, maxValue); }
        }
        return new TerminationBooleanTuple(verticalSuccess, angleSuccess);
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

    public static Action combineCoupledActions(Action landerAction, Action stabilizerAction) {
        return new Action(landerAction.getDouble("thrust"), stabilizerAction.getDouble("gimbalX"), stabilizerAction.getDouble("gimbalY"));
    }

    public static Action combineCoupledTripleActions(Action landerAction, Action gimbalXAction, Action gimbalYAction) {
        return new Action(landerAction.getDouble("thrust"), gimbalXAction.getDouble("gimbalX"), gimbalYAction.getDouble("gimbalY"));
    }

    private int[] getIndecesFromDefinition(HashMap<String, HashMap> MDPDefinition){
        return (int[])MDPDefinition.get("indeces").get("indeces");
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
