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
        constructorCode(methods);
    }

    private void constructorCode(LinkedHashMap<String, MDPDefinition> methods) {
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            MDPDefinition definition = entry.getValue();

            ValueFunction valueFunction = null;
            if (definition.valueFunction != null)
                valueFunction = definition.valueFunction;
            else {
                if (definition.tryToReadFromFile) {
                    boolean readSuccess = RLObjectFileStore.getInstance().tryToReadActionValueFunctionFromDefinition(definition);
                    if (readSuccess)
                        valueFunction = definition.valueFunction;
                }
            }

            if (valueFunction == null) {
                if (ValueFunction.isReasonableTableSize(definition.indexProduct))
                    valueFunction = new ValueFunction(allocateNewValueFunctionTable(definition.indexProduct));
                else {
                    valueFunction = new ValueFunction(new HashMap<>());
                }
            }


            definition.setValueFunction(valueFunction);
            for (ModelBaseImplementation model: definition.models)
                model.setValueFunctionTable(this);
            definition.tryToReadFromFile = true;
        }
    }

    public void resetValueFunctionTable(MDPDefinition[] definitions) {
        for (MDPDefinition definition : definitions) {
            definition.setValueFunction(null);
            definition.tryToReadFromFile = false;
        }
    }

    public static float[] allocateNewValueFunctionTable(int size) {
        System.out.println("Allocating stateSpace: " + size);
        return new float[size];
    }

    public float get(StateActionTuple stateActionTuple) {
        ValueFunction valueFunction = stateActionTuple.state.definition.valueFunction;
        int index = MDPDefinition.computeIndex(stateActionTuple);
        valueFunction.lock(index);
        float result = valueFunction.get(index);
        valueFunction.unlock(index);
        return result;
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        ValueFunction valueFunction = stateActionTuple.state.definition.valueFunction;
        int index = MDPDefinition.computeIndex(stateActionTuple);
        valueFunction.lock(index);
        valueFunction.put(index, newValue);
        valueFunction.unlock(index);
        return newValue;
    }

    public int getIndexAndLock(StateActionTuple stateActionTuple) {
        int index = MDPDefinition.computeIndex(stateActionTuple);
        stateActionTuple.state.definition.valueFunction.lock(index);
        return index;
    }

    public void setValueAtIndexAndUnlock(StateActionTuple stateActionTuple, int index, float value) {
        stateActionTuple.state.definition.valueFunction.put(index, value);
        stateActionTuple.state.definition.valueFunction.unlock(index);
    }

    public int[] getIndecesAndLockAll(ArrayList<StateActionTuple> SA, HashSet<Integer> lockedIndeces) {
        // DEADLOCK IF SA indeces ARE NOT UNIQUE
        int[] indeces = new int[SA.size()];
        for (int i = 0; i < SA.size(); i++) {
            StateActionTuple stateActionTuple = SA.get(i);
            indeces[i] = MDPDefinition.computeIndex(stateActionTuple);
        }
        // lock at last possible moment
        ValueFunction valueFunction = SA.get(0).state.definition.valueFunction;
        valueFunction.lockMainLock();
        for (int i = 0; i < SA.size(); i++) {
            int index = indeces[i];
            if (!lockedIndeces.contains(index)) {  // don't double lock
                valueFunction.lock(index);
                lockedIndeces.add(index);
            }
        }
        valueFunction.unlockMainLock();
        return indeces;
    }

    public void setValueAtIndecesAndUnlockAll(MDPDefinition definition, HashSet<Integer> unlockIndeces, int[] indeces, float[] values) {
        ValueFunction valueFunction = definition.valueFunction;
        for (int i = 0; i < indeces.length; i++) {
            valueFunction.put(indeces[i], values[i]);
        }
        // unlock on completion - MUST BE AFTER ALL UPDATES
        for (int i = 0; i < indeces.length; i++) {
            int index = indeces[i];
            if (unlockIndeces.contains(index)) {
                valueFunction.unlock(index);
                unlockIndeces.remove(index);
            }
        }
    }
}
