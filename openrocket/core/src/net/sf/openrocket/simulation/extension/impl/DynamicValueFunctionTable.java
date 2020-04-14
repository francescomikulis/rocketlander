package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class DynamicValueFunctionTable {
    public static float get(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] valueFunctionTable = map.getValueFunctionArray();
        return valueFunctionTable[computeIndex(indeces, map, stateActionTuple, generalDefinition)];
    }
    public static float getLander(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] landerValueFunctionTable = map.getLanderValueFunctionArray();
        return landerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, landerDefinition)];
    }
    public static float getStabilizer(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] stabilizerValueFunctionTable = map.getStabilizerValueFunctionArray();
        return stabilizerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stabilizerDefinition)];
    }
    public static float getReaching(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] reachingValueFunctionTable = map.getReacherValueFunctionArray();
        return reachingValueFunctionTable[computeIndex(indeces, map, stateActionTuple, landerDefinition)];
    }

    public static float put(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] valueFunctionTable = map.getValueFunctionArray();
        valueFunctionTable[computeIndex(indeces, map, stateActionTuple, generalDefinition)] = newValue;
        return newValue;
    }
    public static float putLander(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] landerValueFunctionTable = map.getLanderValueFunctionArray();
        landerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, landerDefinition)] = newValue;
        return newValue;
    }
    public static float putStabilizer(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] stabilizerValueFunctionTable = map.getStabilizerValueFunctionArray();
        stabilizerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stabilizerDefinition)] = newValue;
        return newValue;
    }
    public static float putReaching(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] reacherValueFunctionTable = map.getReacherValueFunctionArray();
        reacherValueFunctionTable[computeIndex(indeces, map, stateActionTuple, reacherDefinition)] = newValue;
        return newValue;
    }


    private static int computeIndex(int indeces[], OptimizedMap map, StateActionTuple stateActionTuple, HashMap<String, HashMap> MDPDefinition) {
        int index = 0;
        int currentSize = 0;
        int product = OptimizedMap.indexProduct(indeces);
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String stateField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int currentValue = (int) state.get(stateField);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
        }
        for (Object entryObject : MDPDefinition.get("actionDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String actionField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int currentValue = (int) action.get(actionField);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
        }
        return index;
    }

    public static float[] callAllocation(int size) {
        return new float[size];
    }
}
