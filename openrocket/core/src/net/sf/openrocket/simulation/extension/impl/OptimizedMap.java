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
    5: gimbleY
    6: gimbleZ

    --- Action ---
    7: thrust
    8: gimbleY
    9: gimbleZ
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
    private int[] indeces;
    private int[] landerIndeces;
    private int[] stabilizerIndeces;
    private int[] reacherIndeces;

    //public HashMap<String, HashMap> boundaryMapMin = new HashMap<>();
    //public HashMap<String, HashMap> boundaryMapMax = new HashMap<>();

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
            newValueFunctionTable = allocateNewValueFunctionTable();
        this.valueFunctionTable = newValueFunctionTable;
    }

    public OptimizedMap(float[] newLanderValueFunctionTable, float[] newReacherValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCoupled(newLanderValueFunctionTable, newReacherValueFunctionTable, newStabilizerValueFunctionTable);
    }

    private void constructorCoupled(float[] newLanderValueFunctionTable, float[] newReacherValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newLanderValueFunctionTable == null)
            newLanderValueFunctionTable = allocateNewLanderValueFunctionTable();
        this.landerValueFunctionTable = newLanderValueFunctionTable;
        if (newStabilizerValueFunctionTable == null)
            newStabilizerValueFunctionTable = allocateNewStabilizerValueFunctionTable();
        this.stabilizerValueFunctionTable = newStabilizerValueFunctionTable;
        if (newReacherValueFunctionTable == null)
            newReacherValueFunctionTable = allocateNewReacherValueFunctionTable();
        this.reacherValueFunctionTable = newReacherValueFunctionTable;
    }

    private void constructorCode() {
        addIntegersToDefinition(generalDefinition);
        addIntegersToDefinition(landerDefinition);
        addIntegersToDefinition(reacherDefinition);
        addIntegersToDefinition(stabilizerDefinition);

        indeces = generateIndex(generalDefinition);
        landerIndeces = generateIndex(landerDefinition);
        stabilizerIndeces = generateIndex(stabilizerDefinition);
        reacherIndeces = generateIndex(reacherDefinition);
    }


    public float getLander(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getLander(this, landerIndeces, stateActionTuple);
    }

    public float getStabilizer(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getStabilizer(this, stabilizerIndeces, stateActionTuple);
    }

    public float getReacher(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getReaching(this, reacherIndeces, stateActionTuple);
    }

    public float get(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.get(this, indeces, stateActionTuple);
    }

    public float putLander(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putLander(this, landerIndeces, stateActionTuple, newValue);
    }

    public float putStabilizer(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putStabilizer(this, stabilizerIndeces, stateActionTuple, newValue);
    }

    public float putReacher(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putReaching(this, reacherIndeces, stateActionTuple, newValue);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.put(this, indeces, stateActionTuple, newValue);
    }

    public static boolean equivalentStateLander(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (Object objectField : landerDefinition.get("stateDefinition").keySet()) {
            String stateField = (String)objectField;
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }

    public static boolean equivalentStateStabilizer(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (Object objectField : stabilizerDefinition.get("stateDefinition").keySet()) {
            String stateField = (String)objectField;
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }

    public static boolean equivalentStateReacher(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (Object objectField : reacherDefinition.get("stateDefinition").keySet()) {
            String stateField = (String)objectField;
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (Object objectField : generalDefinition.get("stateDefinition").keySet()) {
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

    private float[] allocateNewValueFunctionTable() {
        System.out.println("Allocating stateSpace: " + indexProduct(indeces));
        // return (float[][][][][][][][][][]) DynamicValueFunctionTable.callAllocation(indeces);
        return DynamicValueFunctionTable.callAllocation(indexProduct(indeces));
    }

    private float[] allocateNewLanderValueFunctionTable() {
        System.out.println("Allocating stateSpace: " + indexProduct(landerIndeces));
        // return (float[][][][]) DynamicValueFunctionTable.callAllocation(indeces);
        return DynamicValueFunctionTable.callAllocation(indexProduct(landerIndeces));
    }

    private float[] allocateNewStabilizerValueFunctionTable() {
        System.out.println("Allocating stateSpace: " + indexProduct(stabilizerIndeces));
        // return (float[][][][][][][]) DynamicValueFunctionTable.callAllocation(indeces);
        return DynamicValueFunctionTable.callAllocation(indexProduct(stabilizerIndeces));
    }

    private float[] allocateNewReacherValueFunctionTable() {
        System.out.println("Allocating stateSpace: " + indexProduct(reacherIndeces));
        // return (float[][][][][][][]) DynamicValueFunctionTable.callAllocation(indeces);
        return DynamicValueFunctionTable.callAllocation(indexProduct(reacherIndeces));
    }

    private int[] generateIndex(HashMap<String, HashMap> MDPDefinition){
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
        return indeces;
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
        return new Action(landerAction.getDouble("thrust"), stabilizerAction.getDouble("gimbleY"), stabilizerAction.getDouble("gimbleZ"));
    }

    public static Action combineCoupledTripleActions(Action landerAction, Action gimbalXAction, Action gimbalYAction) {
        return new Action(landerAction.getDouble("thrust"), gimbalXAction.getDouble("gimbleY"), gimbalYAction.getDouble("gimbleZ"));
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
