package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionConstants.MAX_HALF_CIRCLE;
import static net.sf.openrocket.simulation.extension.impl.StateActionConstants.getConstant;
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
    private int[] indeces;
    private int[] landerIndeces;
    private int[] stabilizerIndeces;

    public HashMap<String, Integer> boundaryMapMin = new HashMap<>();
    public HashMap<String, Integer> boundaryMapMax = new HashMap<>();
    public HashMap<String, Method> constants = new HashMap<>();

    public OptimizedMap() {
        if (mapMethod == MapMethod.Traditional)
            constructorTraditional(null);
        else if (mapMethod == MapMethod.Coupled)
            constructorCoupled(null, null);
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

    public OptimizedMap(float[] newLanderValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCoupled(newLanderValueFunctionTable, newStabilizerValueFunctionTable);
    }

    private void constructorCoupled(float[] newLanderValueFunctionTable, float[] newStabilizerValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newLanderValueFunctionTable == null)
            newLanderValueFunctionTable = allocateNewLanderValueFunctionTable();
        this.landerValueFunctionTable = newLanderValueFunctionTable;
        if (newStabilizerValueFunctionTable == null)
            newStabilizerValueFunctionTable = allocateNewStabilizerValueFunctionTable();
        this.stabilizerValueFunctionTable = newStabilizerValueFunctionTable;
    }

    private void constructorCode() {
        HashSet<String> allFields = combineAllFields();

        // generate low minimum values
        StateActionTuple stateActionTuple = new StateActionTuple(null, null);
        State state = new State(null);
        for(String field: allFields) {
            String minField = "MIN_" + field.toUpperCase();
            double value = (double) getConstant(minField);
            if (field.equals("angleX") || field.equals("gimbleY"))
                value = 0.0f;
            int minValue = (int)state.setDouble(field, value).get(field);
            boundaryMapMin.put(field, minValue);
        }
        for(String field: allFields) {
            String maxField = "MAX_" + field.toUpperCase();
            double value = (double) getConstant(maxField);
            if (field.equals("angleX") || field.equals("gimbleY"))
                value = 2 * MAX_HALF_CIRCLE - 0.00001f;
            int maxValue = (int)state.setDouble(field, value).get(field);
            boundaryMapMax.put(field, maxValue);
        }

        indeces = generateIndex(stateDefinition, actionDefinition);
        landerIndeces = generateIndex(stateDefinitionLanding, actionDefinitionLanding);
        stabilizerIndeces = generateIndex(stateDefinitionStabilizing, actionDefinitionStabilizing);
    }


    public float getLander(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getLanding(this, landerIndeces, stateActionTuple);
    }

    public float getStabilizer(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getStabilizing(this, stabilizerIndeces, stateActionTuple);
    }

    public float get(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.get(this, indeces, stateActionTuple);
    }

    public float putLander(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putLanding(this, landerIndeces, stateActionTuple, newValue);
    }

    public float putStabilizer(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putStabilizing(this, stabilizerIndeces, stateActionTuple, newValue);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.put(this, indeces, stateActionTuple, newValue);
    }

    public static boolean equivalentStateLander(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (String stateField: stateDefinitionLanding) {
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }

    public static boolean equivalentStateStabilizer(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (String stateField: stateDefinitionStabilizing) {
            equivalent = equivalent && state_a.get(stateField) == state_b.get(stateField);
        }
        return equivalent;
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = true;
        for (String stateField: stateDefinition) {
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

    public boolean checkBounds(State state) {
        return checkBounds(new StateActionTuple(state, new Action(0, 0, 0)));
    }

    public boolean checkBounds(StateActionTuple stateActionTuple) {
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        for (String stateField: stateDefinition) {
            int currentValue = (int)state.get(stateField);
            if ((currentValue < getMinField(stateField)) || (currentValue > getMaxField(stateField))) return false;
        }
        for (String actionField: actionDefinition) {
            int currentValue = (int)action.get(actionField);
            if ((currentValue < getMinField(actionField)) || (currentValue > getMaxField(actionField))) return false;
        }
        return true;
    }

    public int getMinField(String baseField) {
        return boundaryMapMin.get(baseField);
    }

    public int getMaxField(String baseField) {
        return boundaryMapMax.get(baseField);
    }

    public TerminationBooleanTuple alterTerminalStateIfFailure(State state) {
        boolean verticalSuccess = true;
        boolean angleSuccess = true;

        if (state == null) return new TerminationBooleanTuple(true, true);
        for (String landingField: stateDefinitionLanding) {
            int minValue = getMinField(landingField);
            int maxValue = getMaxField(landingField);
            int currentValue = (int)state.get(landingField);
            if (currentValue < minValue) { verticalSuccess = false; state.set(landingField, minValue); }
            if (currentValue > maxValue) { verticalSuccess = false; state.set(landingField, maxValue); }
        }
        for (String stabilizingField: stateDefinitionStabilizing) {
            int minValue = getMinField(stabilizingField);
            int maxValue = getMaxField(stabilizingField);
            int currentValue = (int)state.get(stabilizingField);
            if (currentValue < minValue) { angleSuccess = false; state.set(stabilizingField, minValue); }
            if (currentValue > maxValue) { angleSuccess = false; state.set(stabilizingField, maxValue); }
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

    private int[] generateIndex(ArrayList<String> stateDefinition, ArrayList<String> actionDefinition){
        int[] indeces = new int[stateDefinition.size() + actionDefinition.size()];
        int index = 0;
        for (String stateField: stateDefinition) {
            indeces[index] = (getMaxField(stateField) - getMinField(stateField) + 1);
            index += 1;
        }
        for (String actionField: actionDefinition) {
            indeces[index] = (getMaxField(actionField) - getMinField(actionField) + 1);
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

    public static Action combineCoupledActions(Action landerAction, Action stabilizerAction) {
        return new Action(landerAction.getDouble("thrust"), stabilizerAction.getDouble("gimbleY"), stabilizerAction.getDouble("gimbleZ"));
    }

    private HashSet<String> combineAllFields() {
        HashSet<String> allFields = new HashSet<>();
        allFields.addAll(stateDefinition); allFields.addAll(actionDefinition);
        allFields.addAll(stateDefinitionLanding); allFields.addAll(actionDefinitionLanding);
        allFields.addAll(stateDefinitionStabilizing); allFields.addAll(actionDefinitionStabilizing);
        return allFields;
    }
}
