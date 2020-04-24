package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

public class DynamicValueFunctionTable {
    public static float get(OptimizedMap map, StateActionTuple stateActionTuple) {
        String definitionName = (String)stateActionTuple.state.definition.get("meta").get("name");
        float[] valueFunctionTable = getCorrectTable(map, definitionName);
        int index = computeIndex(stateActionTuple);
        return valueFunctionTable[index];
    }
    public static float put(OptimizedMap map, StateActionTuple stateActionTuple, float newValue) {
        String definitionName = (String)stateActionTuple.state.definition.get("meta").get("name");
        float[] valueFunctionTable = getCorrectTable(map, definitionName);
        int index = computeIndex(stateActionTuple);
        valueFunctionTable[index] = newValue;
        return newValue;
    }
    
    private static float[] getCorrectTable(OptimizedMap map, String definitionName){
        float[] valueFunctionTable = new float[0];
        if (definitionName.equals("general"))
            valueFunctionTable = map.getValueFunctionArray();
        else if (definitionName.equals("lander"))
            valueFunctionTable = map.getLanderValueFunctionArray();
        else if (definitionName.equals("reacher"))
            valueFunctionTable = map.getReacherValueFunctionArray();
        else if (definitionName.equals("stabilizer"))
            valueFunctionTable = map.getStabilizerValueFunctionArray();
        else
            System.out.println("VALUE FUNCTION TABLE GET DID NOT WORK!");
        return valueFunctionTable;
    }

    private static int computeIndex(StateActionTuple stateActionTuple) {
        return computeIndexState(stateActionTuple.state) + computeIndexAction(stateActionTuple.action);
    }

    public static int computeIndexState(State state) {
        int index = 0;
        int currentSize = 0;
        HashMap<String, LinkedHashMap> MDPDefinition = state.definition;
        int indeces[] = ((int[])(MDPDefinition.get("indeces").get("indeces")));
        int product = OptimizedMap.indexProduct(indeces);

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String stateField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) state.get(stateField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
            product /= indeces[currentSize];
            index += (currentValue - minValue) * product;
            currentSize += 1;
        }
        return index;
    }

    public static int computeIndexAction(Action action) {
        int index = 0;
        int currentSize = 0;
        HashMap<String, LinkedHashMap> MDPDefinition = action.definition;
        int indeces[] = ((int[])(MDPDefinition.get("indeces").get("indeces")));
        int product = OptimizedMap.indexProduct(indeces);

        for (Object entryObject : MDPDefinition.get("stateDefinitionIntegers").entrySet()) {
            product /= indeces[currentSize];
            currentSize += 1;
        }

        for (Object entryObject : MDPDefinition.get("actionDefinitionIntegers").entrySet()) {
            Map.Entry<String, int[]> entry = (Map.Entry<String, int[]>) entryObject;
            String actionField = entry.getKey();
            int[] minMax = entry.getValue();
            int minValue = minMax[0];
            int maxValue = minMax[1];
            int currentValue = (int) action.get(actionField);
            currentValue = Math.max(currentValue, minValue);
            currentValue = Math.min(currentValue, maxValue);
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
