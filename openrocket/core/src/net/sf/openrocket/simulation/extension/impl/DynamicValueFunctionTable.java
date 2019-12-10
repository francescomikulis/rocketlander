package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class DynamicValueFunctionTable {
    public static float get(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] valueFunctionTable = map.getValueFunctionArray();
        return valueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinition, actionDefinition)];
    }
    public static float getLanding(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] landerValueFunctionTable = map.getLanderValueFunctionArray();
        return landerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinitionLanding, actionDefinitionLanding)];
    }
    public static float getStabilizing(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple) {
        float[] stabilizerValueFunctionTable = map.getStabilizerValueFunctionArray();
        return stabilizerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinitionStabilizing, actionDefinitionStabilizing)];
    }

    public static float put(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] valueFunctionTable = map.getValueFunctionArray();
        valueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinition, actionDefinition)] = newValue;
        return newValue;
    }
    public static float putLanding(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] landerValueFunctionTable = map.getLanderValueFunctionArray();
        landerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinitionLanding, actionDefinitionLanding)] = newValue;
        return newValue;
    }
    public static float putStabilizing(OptimizedMap map, int[] indeces, StateActionTuple stateActionTuple, float newValue) {
        float[] stabilizerValueFunctionTable = map.getStabilizerValueFunctionArray();
        stabilizerValueFunctionTable[computeIndex(indeces, map, stateActionTuple, stateDefinitionStabilizing, actionDefinitionStabilizing)] = newValue;
        return newValue;
    }


    private static int computeIndex(int indeces[], OptimizedMap map, StateActionTuple stateActionTuple, ArrayList<String> stateDefinitions, ArrayList<String> actionDefinitions) {
        int index = 0;
        int currentSize = 0;
        int product = OptimizedMap.indexProduct(indeces);
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        for (String stateField: stateDefinitions) {
            int currentValue = (int)state.get(stateField);
            product /= indeces[currentSize];
            index += (currentValue - map.getMinField(stateField)) * product;
            currentSize += 1;
        }
        for (String actionField: actionDefinitions) {
            int currentValue = (int)action.get(actionField);
            product /= indeces[currentSize];
            index += (currentValue - map.getMinField(actionField)) * product;
            currentSize += 1;
        }
        return index;
    }

    public static float[] callAllocation(int size) {
        return new float[size];
    }
}
