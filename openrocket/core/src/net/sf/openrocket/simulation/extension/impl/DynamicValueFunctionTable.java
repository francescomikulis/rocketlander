package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

import java.lang.reflect.Field;

public class DynamicValueFunctionTable {
    public static float get(OptimizedMap map, Object valueFunctionTable, State state, Action action, ArrayList<String> stateDefinitions, ArrayList<String> actionDefinitions) {
        DynamicValueFunctionTable self = new DynamicValueFunctionTable();

        int currentSize = stateDefinitions.size() + actionDefinitions.size();

        Object baseTable = (Object)valueFunctionTable;

        for (String stateField: stateDefinitions) {
            try {
                Field field = state.getClass().getField(stateField);
                field.setAccessible(true);
                int currentValue = field.getInt(state);
                int index = currentValue - getMinField(map, stateField);

                Method method = self.getClass().getDeclaredMethod("get" + currentSize, Object.class, int.class);
                method.setAccessible(true);
                baseTable = method.invoke(null, baseTable, index);
                currentSize -= 1;
            } catch (NoSuchFieldException | IllegalAccessException | NoSuchMethodException | InvocationTargetException exc) {
                exc.printStackTrace();
            }
        }

        for (String actionField: actionDefinitions) {
            try {
                Field field = action.getClass().getField(actionField);
                field.setAccessible(true);
                int currentValue = field.getInt(action);
                int index = currentValue - getMinField(map, actionField);

                Method method = self.getClass().getDeclaredMethod("get" + currentSize, Object.class, int.class);
                method.setAccessible(true);
                baseTable = method.invoke(null, baseTable, index);
                currentSize -= 1;
            } catch (NoSuchFieldException | IllegalAccessException | NoSuchMethodException | InvocationTargetException exc) {
                exc.printStackTrace();
            }
        }

        if (currentSize != 0) {
            System.out.println("BAD BAD BAD BAD SIZE IS NOT ZERO AND SHOULD BE!");
        }

        return (float)baseTable;
    }
    private static int getMinField(OptimizedMap map, String baseField) {
        String minField = "min" + baseField.substring(0, 1).toUpperCase() + baseField.substring(1);
        int minValue = 0;
        try {
            Field field = map.getClass().getDeclaredField(minField);
            field.setAccessible(true);
            minValue = field.getInt(map);
        } catch (IllegalAccessException | NoSuchFieldException exc) {
            exc.printStackTrace();
        }
        return minValue;
    }

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
}
