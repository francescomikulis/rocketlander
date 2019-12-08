package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

public class DynamicValueFunctionTable {
    public static float get(OptimizedMap map, State state, Action action) {
        return logic("get", map, map.getValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinition, ModelBaseImplementation.actionDefinition, 0.0f);
    }
    public static float getLanding(OptimizedMap map, State state, Action action) {
        return logic("get", map, map.getLanderValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinitionLanding, ModelBaseImplementation.actionDefinitionLanding, 0.0f);
    }
    public static float getStabilizing(OptimizedMap map, State state, Action action) {
        return logic("get", map, map.getStabilizerValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinitionStabilizing, ModelBaseImplementation.actionDefinitionStabilizing, 0.0f);
    }

    public static float put(OptimizedMap map, State state, Action action, float newValue) {
        return logic("put", map, map.getValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinition, ModelBaseImplementation.actionDefinition, newValue);
    }
    public static float putLanding(OptimizedMap map, State state, Action action, float newValue) {
        return logic("put", map, map.getLanderValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinitionLanding, ModelBaseImplementation.actionDefinitionLanding, newValue);
    }
    public static float putStabilizing(OptimizedMap map, State state, Action action, float newValue) {
        return logic("put", map, map.getStabilizerValueFunctionArray(), state, action, ModelBaseImplementation.stateDefinitionStabilizing, ModelBaseImplementation.actionDefinitionStabilizing, newValue);
    }

    private static float logic(String methodName, OptimizedMap map, Object valueFunctionTable, State state, Action action, ArrayList<String> stateDefinitions, ArrayList<String> actionDefinitions, float newValue) {
        DynamicValueFunctionTable self = new DynamicValueFunctionTable();
        int maxSize = stateDefinitions.size() + actionDefinitions.size();
        int currentSize = 0;
        int indeces[] = new int[maxSize];

        for (String stateField: stateDefinitions) {
            int currentValue = state.getField(stateField);
            indeces[currentSize] = currentValue - map.getMinField(stateField);
            currentSize += 1;
        }

        for (String actionField: actionDefinitions) {
            int currentValue = action.getField(actionField);
            indeces[currentSize] = currentValue - map.getMinField(actionField);
            currentSize += 1;
        }

        if (maxSize - currentSize != 0) {
            System.out.println("BAD BAD BAD BAD SIZE IS NOT ZERO AND SHOULD BE!");
        }

        return (float)self.callGetWithNumber(methodName, map, self, valueFunctionTable, indeces, newValue);
    }

    /*
    private static float[][][][][][][][][][][][] get13(Object thirteen, int val){ return ((float [][][][][][][][][][][][][])thirteen)[val]; }
    private static float[][][][][][][][][][][] get12(Object twelve, int val){ return ((float [][][][][][][][][][][][])twelve)[val]; }
    private static float[][][][][][][][][][] get11(Object eleven, int val){ return ((float [][][][][][][][][][][])eleven)[val]; }
    private static float[][][][][][][][][] get10(Object ten, int val){ return ((float [][][][][][][][][][])ten)[val]; }
    private static float[][][][][][][][] get9(Object nine, int val){ return ((float [][][][][][][][][])nine)[val]; }
    private static float[][][][][][][] get8(Object eight, int val){ return ((float [][][][][][][][])eight)[val]; }
    private static float[][][][][][] get7(Object seven, int val){ return ((float [][][][][][][])seven)[val]; }
    private static float[][][][][] get6(Object six, int val){ return ((float [][][][][][])six)[val]; }
    private static float[][][][] get5(Object five, int val){ return ((float [][][][][])five)[val]; }
    private static float[][][] get4(Object four, int val){ return ((float [][][][])four)[val]; }
    private static float[][] get3(Object three, int val){ return ((float [][][])three)[val]; }
    private static float[] get2(Object two, int val){ return ((float [][])two)[val]; }
    private static float get1(Object one, int val){ return ((float[])one)[val]; }
     */

    private static float get13(boolean isGet, Object thirteen, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][][][][][][])thirteen)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]][indeces[8]][indeces[9]][indeces[10]][indeces[11]][indeces[12]];
        ((float[][][][][][][][][][][][][]) thirteen)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]][indeces[8]][indeces[9]][indeces[10]][indeces[11]][indeces[12]] = newValue;
        return newValue;
    }
    private static float get12(boolean isGet, Object twelve, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][][][][][])twelve)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]][indeces[8]][indeces[9]][indeces[10]][indeces[11]];
        ((float[][][][][][][][][][][][]) twelve)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]][indeces[8]][indeces[9]][indeces[10]][indeces[11]] = newValue;
        return newValue;
    }
    private static float get11(boolean isGet, Object eleven, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][][][][])eleven)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]][indeces[8]][indeces[9]][indeces[10]];
        ((float[][][][][][][][][][][]) eleven)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]][indeces[8]][indeces[9]][indeces[10]] = newValue;
        return newValue;
    }
    private static float get10(boolean isGet, Object ten, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][][][])ten)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]][indeces[8]][indeces[9]];
        ((float[][][][][][][][][][]) ten)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]][indeces[8]][indeces[9]] = newValue;
        return newValue;
    }
    private static float get9(boolean isGet, Object nine, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][][])nine)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]][indeces[8]];
        ((float[][][][][][][][][]) nine)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]][indeces[8]] = newValue;
        return newValue;
    }
    private static float get8(boolean isGet, Object eight, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][][])eight)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                    [indeces[7]];
        ((float[][][][][][][][]) eight)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]]
                [indeces[7]] = newValue;
        return newValue;
    }
    private static float get7(boolean isGet, Object seven, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][][])seven)
                    [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]];
        ((float[][][][][][][]) seven)
                [indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]] = newValue;
        return newValue;
    }
    private static float get6(boolean isGet, Object six, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][][])six)[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]];
        ((float[][][][][][]) six)[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]] = newValue;
        return newValue;
    }
    private static float get5(boolean isGet, Object five, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][][])five)[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]];
        ((float[][][][][]) five)[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]] = newValue;
        return newValue;
    }
    private static float get4(boolean isGet, Object four, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][][])four)[indeces[0]][indeces[1]][indeces[2]][indeces[3]];
        ((float[][][][]) four)[indeces[0]][indeces[1]][indeces[2]][indeces[3]] = newValue;
        return newValue;
    }
    private static float get3(boolean isGet, Object three, int[] indeces, float newValue){
        if (isGet)
            return ((float[][][])three)[indeces[0]][indeces[1]][indeces[2]];
        ((float[][][]) three)[indeces[0]][indeces[1]][indeces[2]] = newValue;
        return newValue;
    }
    private static float get2(boolean isGet, Object two, int[] indeces, float newValue){
        if (isGet)
            return ((float[][])two)[indeces[0]][indeces[1]];
        ((float[][]) two)[indeces[0]][indeces[1]] = newValue;
        return newValue;
    }
    private static float get1(boolean isGet, Object one, int[] indeces, float newValue){
        if (isGet)
            return ((float[])one)[indeces[0]];
        ((float[]) one)[indeces[0]] = newValue;
        return newValue;
    }



    private static void put(Object one, int index, float val){ ((float[])one)[index] = val; }

    /*
    private static Object callGetWithNumber(DynamicValueFunctionTable self, Object baseTable, int currentSize, int index) {
        try {
            Method method = self.getClass().getDeclaredMethod("get" + currentSize, Object.class, int.class);
            method.setAccessible(true);
            baseTable = method.invoke(null, baseTable, index);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException exc) {
            exc.printStackTrace();
        }
        return baseTable;
    }
     */

    private float callGetWithNumber(String methodName, OptimizedMap map, DynamicValueFunctionTable self, Object baseTable, int[] indeces, float newValue) {
        boolean isGet = methodName.equals("get");
        float result = 0.0f;
        String actualMethodName = "get" + indeces.length;
        Method method = null;
        if (map.methodMap.containsKey(actualMethodName)) method = map.methodMap.get(actualMethodName);
        try {
            if (method == null) {
                method = self.getClass().getDeclaredMethod(actualMethodName, boolean.class, Object.class, int[].class, float.class);
                method.setAccessible(true);
                map.methodMap.put(actualMethodName, method);
            }
            result = (float) method.invoke(null, isGet, baseTable, indeces, newValue);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException exc) {
            exc.printStackTrace();
        }

        return result;
    }

    private static float[][][][][][][][] allocate8(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]][indeces[7]];
    }
    private static float[][][][][][][] allocate7(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]][indeces[6]];
    }
    private static float[][][][][][] allocate6(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]][indeces[5]];
    }
    private static float[][][][][] allocate5(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]][indeces[3]][indeces[4]];
    }
    private static float[][][][] allocate4(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]][indeces[3]];
    }
    private static float[][][] allocate3(int[] indeces){
        return new float[indeces[0]][indeces[1]][indeces[2]];
    }

    public static Object callAllocation(int[] indeces) {
        DynamicValueFunctionTable self = new DynamicValueFunctionTable();
        String actualMethodName = "allocate" + indeces.length;
        Object result = null;
        try {
            Method method = self.getClass().getDeclaredMethod(actualMethodName, int[].class);
            method.setAccessible(true);
            result = method.invoke(null, indeces);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException exc) {
            exc.printStackTrace();
        }
        return result;
    }
}
