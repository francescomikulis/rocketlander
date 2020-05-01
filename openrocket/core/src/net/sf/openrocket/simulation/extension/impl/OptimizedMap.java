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

            float[] valueFunctionTable = null;
            if (definition.valueFunction != null)
                valueFunctionTable = definition.valueFunction;
            else {
                if (definition.tryToReadFromFile) {
                    boolean readSuccess = RLObjectFileStore.getInstance().tryToReadActionValueFunctionFromDefinition(definition);
                    if (readSuccess)
                        valueFunctionTable = definition.valueFunction;
                }
            }

            if (valueFunctionTable == null)
                valueFunctionTable = allocateNewValueFunctionTable(definition.indexProduct);


            definition.setValueFunction(valueFunctionTable);
            for (ModelBaseImplementation model: definition.models)
                model.setValueFunctionTable(this);
            checkTableValues(definition);
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

    public int getIndexAndLock(StateActionTuple stateActionTuple) {
        int index = MDPDefinition.computeIndex(stateActionTuple);
        stateActionTuple.state.definition.locks[index].lock();
        return index;
    }

    public void setValueAtIndexAndUnlock(StateActionTuple stateActionTuple, int index, float value) {
        stateActionTuple.state.definition.valueFunction[index] = value;
        stateActionTuple.state.definition.locks[index].unlock();
    }

    public int[] getIndecesAndLockAll(ArrayList<StateActionTuple> SA, HashSet<Integer> lockedIndeces) {
        // DEADLOCK IF SA indeces ARE NOT UNIQUE
        int[] indeces = new int[SA.size()];
        for (int i = 0; i < SA.size(); i++) {
            StateActionTuple stateActionTuple = SA.get(i);
            indeces[i] = MDPDefinition.computeIndex(stateActionTuple);
        }
        // lock at last possible moment
        SA.get(0).state.definition.mainLock.lock();
        final ReentrantLock[] locks = SA.get(0).state.definition.locks;
        for (int i = 0; i < SA.size(); i++) {
            int index = indeces[i];
            if (!lockedIndeces.contains(index)) {  // don't double lock
                locks[index].lock();
                lockedIndeces.add(index);
            }
        }
        SA.get(0).state.definition.mainLock.unlock();
        return indeces;
    }

    public void setValueAtIndecesAndUnlockAll(MDPDefinition definition, HashSet<Integer> unlockIndeces, int[] indeces, float[] values) {
        final ReentrantLock[] locks = definition.locks;
        final float[] valueFunction = definition.valueFunction;
        for (int i = 0; i < indeces.length; i++) {
            int index = indeces[i];
            valueFunction[index] = values[i];
            // release locks as soon as possible
            if (unlockIndeces.contains(index)) {
                locks[index].unlock();
                unlockIndeces.remove(index);
            }
        }
    }
}
