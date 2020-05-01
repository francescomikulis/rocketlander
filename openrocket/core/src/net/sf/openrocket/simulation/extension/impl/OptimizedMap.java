package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

import java.util.*;
import java.util.concurrent.locks.ReentrantLock;

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
    public OptimizedMap(LinkedHashMap<String, MDPDefinition> methods) {
        constructorCode(methods, false);
    }

    public OptimizedMap(LinkedHashMap<String, MDPDefinition> methods, boolean reset) {
        constructorCode(methods, reset);
    }

    private void constructorCode(LinkedHashMap<String, MDPDefinition> methods, boolean reset) {
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            MDPDefinition definition = entry.getValue();

            float[] valueFunctionTable = null;
            boolean readSuccess = true;
            if (!reset) {
                if (definition.valueFunction != null)
                    valueFunctionTable = definition.valueFunction;
                else {
                    readSuccess = RLObjectFileStore.getInstance().tryToReadActionValueFunctionFromDefinition(definition);
                    if (readSuccess)
                        valueFunctionTable = definition.valueFunction;
                }
            }

            if (reset || !readSuccess)
                valueFunctionTable = allocateNewValueFunctionTable(definition.indexProduct);

            definition.setValueFunction(valueFunctionTable);
            for (ModelBaseImplementation model: definition.models)
                model.setValueFunctionTable(this);
            checkTableValues(definition);
        }
    }

    public void resetValueFunctionTable(MDPDefinition[] definitions) {
        for (MDPDefinition definition : definitions) {
            definition.setValueFunction(null);
        }
    }

    public static float[] allocateNewValueFunctionTable(int size) {
        System.out.println("Allocating stateSpace: " + size);
        return new float[size];
    }

    public void checkTableValues(MDPDefinition definition) {
        float[] table = definition.valueFunction;
        int product = definition.indexProduct;
        for (int i = 0; i < product; i++) {
            if (Float.isNaN(table[i])) {
                System.out.println("NAN IN TABLE!");
            }
        }
    }

    public float get(StateActionTuple stateActionTuple) {
        float[] valueFunctionTable = stateActionTuple.state.definition.valueFunction;
        int index = MDPDefinition.computeIndex(stateActionTuple);
        final ReentrantLock lock = stateActionTuple.state.definition.locks[index];
        lock.lock();
        float result = valueFunctionTable[index];
        lock.unlock();
        return result;
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        float[] valueFunctionTable = stateActionTuple.state.definition.valueFunction;
        int index = MDPDefinition.computeIndex(stateActionTuple);
        final ReentrantLock lock = stateActionTuple.state.definition.locks[index];
        lock.lock();
        valueFunctionTable[index] = newValue;
        lock.unlock();
        return newValue;
    }
}
