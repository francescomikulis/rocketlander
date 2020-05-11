package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.simulation.extension.impl.rocketlander.methods.BaseMethodImplementation;

import java.util.*;

/**

 */

public class ValueFunctionManager {
    public ValueFunctionManager(LinkedHashMap<String, MDPDefinition> methods) {
        constructorCode(methods);
    }

    private void constructorCode(LinkedHashMap<String, MDPDefinition> methods) {
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            MDPDefinition definition = entry.getValue();

            RLValueFunction valueFunction = null;
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
                int size = definition.indexProduct;
                if (RLValueFunction.isReasonableTableSize(size)) {
                    System.out.println("Allocating stateSpace: " + size);
                    valueFunction = new RLValueFunction(new float[size]);
                } else {
                    valueFunction = new RLValueFunction(new HashMap<>());
                }
            }


            definition.setValueFunction(valueFunction);
            for (BaseMethodImplementation model: definition.models)
                model.setValueFunctionManager(this);
            definition.tryToReadFromFile = true;
        }
    }

    public void resetValueFunctionManager(MDPDefinition[] definitions) {
        for (MDPDefinition definition : definitions) {
            definition.setValueFunction(null);
            definition.tryToReadFromFile = false;
        }
    }

    public float get(StateActionTuple stateActionTuple) {
        RLValueFunction valueFunction = stateActionTuple.state.definition.valueFunction;
        int index = MDPDefinition.computeIndex(stateActionTuple);
        valueFunction.lock(index);
        float result = valueFunction.get(index);
        valueFunction.unlock(index);
        return result;
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        RLValueFunction valueFunction = stateActionTuple.state.definition.valueFunction;
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
        RLValueFunction valueFunction = SA.get(0).state.definition.valueFunction;
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
        RLValueFunction valueFunction = definition.valueFunction;
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
