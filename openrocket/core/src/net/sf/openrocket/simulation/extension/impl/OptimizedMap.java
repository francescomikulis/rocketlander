package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.*;
import java.util.function.Function;

import static java.lang.Float.NaN;
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
        System.out.println("Hit constructor code for OptimizedMap");
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
    }

    public float get(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.get(this, stateActionTuple);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.put(this, stateActionTuple, newValue);
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

    public TerminationBooleanTuple getTerminationValidity(CoupledStates coupledStates) {
        boolean verticalSuccess = true;
        boolean angleSuccess = true;

        if (coupledStates == null) return new TerminationBooleanTuple(true, true);

        State landerState = coupledStates.get(0);
        for (Object entryObject : landerState.definition.get("stateDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
            String field = entry.getKey();
            double currentValue = landerState.getDouble(field);
            float[] minMaxPrecision = entry.getValue();
            if (currentValue < minMaxPrecision[0]) { verticalSuccess = false; break; }
            if (currentValue > minMaxPrecision[1]) { verticalSuccess = false; break; }
        }

        State reacherState = coupledStates.get(1);
        for (Object entryObject : reacherState.definition.get("stateDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
            String field = entry.getKey();
            double currentValue = reacherState.getDouble(field);
            float[] minMaxPrecision = entry.getValue();
            if (currentValue < minMaxPrecision[0]) { angleSuccess = false; break; }
            if (currentValue > minMaxPrecision[1]) { angleSuccess = false; break; }
        }

        State stabilizerState = coupledStates.get(2);
        for (Object entryObject : stabilizerState.definition.get("stateDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>) entryObject;
            String field = entry.getKey();
            double currentValue = stabilizerState.getDouble(field);
            float[] minMaxPrecision = entry.getValue();
            if (currentValue < minMaxPrecision[0]) { angleSuccess = false; break; }
            if (currentValue > minMaxPrecision[1]) { angleSuccess = false; break; }
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
        State state = new State(null);
        state.definition = MDPDefinition;
        for (Object entryObject : MDPDefinition.get("stateDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>)entryObject;
            String field = entry.getKey();
            if (field.contains("angle") || field.contains("gimbal")) {
                float precision = entry.getValue()[2];
                if (precision >= 0.1) {
                    float[] definitionValues = entry.getValue();
                    definitionValues[0] *= (Math.PI / 180);
                    definitionValues[1] *= (Math.PI / 180);
                    definitionValues[2] *= (Math.PI / 180);
                }
            }
        }
        for (Object entryObject : MDPDefinition.get("actionDefinition").entrySet()) {
            Map.Entry<String, float[]> entry = (Map.Entry<String, float[]>)entryObject;
            String field = entry.getKey();
            if (field.contains("angle") || field.contains("gimbal")) {
                float precision = entry.getValue()[2];
                if (precision >= 0.1) {
                    float[] definitionValues = entry.getValue();
                    definitionValues[0] *= (Math.PI / 180);
                    definitionValues[1] *= (Math.PI / 180);
                    definitionValues[2] *= (Math.PI / 180);
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
