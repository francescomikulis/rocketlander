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
    public HashMap<String, float[]> valueFunctionTables = new HashMap<>();

    public OptimizedMap(LinkedHashMap<String, ModelBaseImplementation> methods) {
        constructorCode(methods, false);
    }

    public OptimizedMap(LinkedHashMap<String, ModelBaseImplementation> methods, boolean reset) {
        constructorCode(methods, reset);
    }

    private void constructorCode(LinkedHashMap<String, ModelBaseImplementation> methods, boolean reset) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            ModelBaseImplementation method = entry.getValue();
            MDPDefinition definition = method.definition;

            float[] valueFunctionTable = null;
            if ((!reset) && (definition.valueFunction != null)) {
                valueFunctionTable = (float[]) definition.valueFunction;
            }
            if (valueFunctionTable == null) {
                valueFunctionTable = allocateNewValueFunctionTable(definition.indexProduct);
            }

            definition.valueFunction = valueFunctionTable;
            method.setValueFunctionTable(this);
            valueFunctionTables.put(entry.getKey(), valueFunctionTable);
            checkTableValues(definition);
        }
    }

    public void resetValueFunctionTable(LinkedHashMap<String, ModelBaseImplementation> methods) {
        for (Map.Entry<String, ModelBaseImplementation> entry : methods.entrySet()) {
            MDPDefinition definition = entry.getValue().definition;

            definition.valueFunction = null;
            valueFunctionTables.remove(definition.name);
            entry.getValue().removeValueFunctionTable();
        }
    }

    private float[] allocateNewValueFunctionTable(int size) {
        System.out.println("Allocating stateSpace: " + size);
        return new float[size];
    }

    public void checkTableValues(MDPDefinition definition) {
        float[] table = valueFunctionTables.get(definition.name);

        int product = definition.indexProduct;
        for (int i = 0; i < product; i++) {
            if (Float.isNaN(table[i])) {
                System.out.println("NAN IN TABLE!");
            }
        }
    }

    public float get(StateActionTuple stateActionTuple) {
        String definitionName = (String)stateActionTuple.state.definition.name;
        float[] valueFunctionTable = valueFunctionTables.get(definitionName);
        int index = MDPDefinition.computeIndex(stateActionTuple);
        return valueFunctionTable[index];
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        String definitionName = (String)stateActionTuple.state.definition.name;
        float[] valueFunctionTable = valueFunctionTables.get(definitionName);
        int index = MDPDefinition.computeIndex(stateActionTuple);
        valueFunctionTable[index] = newValue;
        return newValue;
    }
}
