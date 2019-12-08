package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import static net.sf.openrocket.simulation.extension.impl.DynamicValueFunctionTable.*;

/**
    valueFunctionTable
        StateActionTuples --> float

    --- State ---
    0: altitude
    1: velocity
    2: thrust
    3: angleX
    4: angleZ
    5: gimbleY
    6: gimbleZ

    --- Action ---
    7: thrust
    8: gimbleY
    9: gimbleZ
 */

public class OptimizedMap {
    enum MapMethod {
        Traditional, Coupled
    }
    public static MapMethod mapMethod = MapMethod.Coupled;

    private float[][][][][][][][][][] valueFunctionTable = null;
    private float[][][][] landerValueFunctionTable = null;
    private float[][][][][][][] stabilizerValueFunctionTable = null;
    private int minAltitude, minPositionX, minPositionY, minVelocity, minTime, minThrust, minAngleX, minAngleZ, minGimbleY, minGimbleZ;
    private int maxAltitude, maxPositionX, maxPositionY, maxVelocity, maxTime, maxThrust, maxAngleX, maxAngleZ, maxGimbleY, maxGimbleZ;

    public HashMap<String, Integer> boundaryMapMin = new HashMap<>();
    public HashMap<String, Integer> boundaryMapMax = new HashMap<>();
    public HashMap<String, Method> methodMap = new HashMap<>();

    public OptimizedMap() {
        if (mapMethod == MapMethod.Traditional)
            constructorTraditional(null);
        else if (mapMethod == MapMethod.Coupled)
            constructorCoupled(null, null);
    }

    public OptimizedMap(float[][][][][][][][][][] newValueFunctionTable) {
        constructorTraditional(newValueFunctionTable);
    }

    private void constructorTraditional(float[][][][][][][][][][] newValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newValueFunctionTable == null)
            newValueFunctionTable = allocateNewValueFunctionTable();
        this.valueFunctionTable = newValueFunctionTable;
    }

    public OptimizedMap(float[][][][] newLanderValueFunctionTable, float[][][][][][][] newStabilizerValueFunctionTable) {
        constructorCoupled(newLanderValueFunctionTable, newStabilizerValueFunctionTable);
    }

    private void constructorCoupled(float[][][][] newLanderValueFunctionTable, float[][][][][][][] newStabilizerValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newLanderValueFunctionTable == null)
            newLanderValueFunctionTable = allocateNewLanderValueFunctionTable();
        this.landerValueFunctionTable = newLanderValueFunctionTable;
        if (newStabilizerValueFunctionTable == null)
            newStabilizerValueFunctionTable = allocateNewStabilizerValueFunctionTable();
        this.stabilizerValueFunctionTable = newStabilizerValueFunctionTable;
    }

    private void constructorCode() {
        // generate low minimum values
        State lowState = new State(null);
        minAltitude = lowState.setAltitude(MIN_ALTITUDE).altitude;
        boundaryMapMin.put("altitude", minAltitude);
        minPositionX = lowState.setAltitude(MIN_POSITION).positionX;
        boundaryMapMin.put("positionX", minPositionX);
        minPositionY = lowState.setAltitude(MIN_POSITION).positionY;
        boundaryMapMin.put("positionY", minPositionY);
        minVelocity = lowState.setVelocity(MIN_VELOCITY).velocity;
        boundaryMapMin.put("velocity", minVelocity);
        minTime = lowState.setTime(MIN_TIME).time;
        boundaryMapMin.put("time", minTime);
        minThrust = lowState.setThrust(MIN_THRUST).thrust;
        boundaryMapMin.put("thrust", minThrust);
        minAngleX = lowState.setAngleX(0).angleX;
        boundaryMapMin.put("angleX", minAngleX);
        minAngleZ = lowState.setAngleZ(MIN_TERMINAL_ORIENTATION_Z).angleZ;
        boundaryMapMin.put("angleZ", minAngleZ);
        minGimbleY = lowState.setGimbleY(0).gimbleY;
        boundaryMapMin.put("gimbleY", minGimbleY);
        minGimbleZ = lowState.setGimbleZ(MIN_GIMBLE_Z).gimbleZ;
        boundaryMapMin.put("gimbleZ", minGimbleZ);
        // generate high maximum values
        State highState = new State(null);
        maxAltitude = highState.setAltitude(MAX_ALTITUDE).altitude;
        boundaryMapMax.put("altitude", maxAltitude);
        maxPositionX = lowState.setAltitude(MAX_POSITION).positionX;
        boundaryMapMax.put("positionX", maxPositionX);
        maxPositionY = lowState.setAltitude(MAX_POSITION).positionY;
        boundaryMapMax.put("positionY", maxPositionY);
        maxVelocity = highState.setVelocity(MAX_VELOCITY).velocity;
        boundaryMapMax.put("velocity", maxVelocity);
        maxTime = highState.setTime(MAX_TIME).time;
        boundaryMapMax.put("time", maxTime);
        maxThrust = highState.setThrust(MAX_THRUST).thrust;
        boundaryMapMax.put("thrust", maxThrust);
        maxAngleX = highState.setAngleX(2 * MAX_HALF_CIRCLE - 0.00001).angleX;
        boundaryMapMax.put("angleX", maxAngleX);
        maxAngleZ = highState.setAngleZ(MAX_TERMINAL_ORIENTATION_Z).angleZ;
        boundaryMapMax.put("angleZ", maxAngleZ);
        maxGimbleY = highState.setGimbleY(2 * MAX_HALF_CIRCLE - 0.00001).gimbleY;
        boundaryMapMax.put("gimbleY", maxGimbleY);
        maxGimbleZ = highState.setGimbleZ(MAX_GIMBLE_Z).gimbleZ;
        boundaryMapMax.put("gimbleZ", maxGimbleZ);
    }

    public float getLander(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getLanding(this, stateActionTuple.state, stateActionTuple.action);
    }

    public float getStabilizer(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.getStabilizing(this, stateActionTuple.state, stateActionTuple.action);
    }

    public float get(StateActionTuple stateActionTuple) {
        return DynamicValueFunctionTable.get(this, stateActionTuple.state, stateActionTuple.action);
    }

    public float putLander(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putLanding(this, stateActionTuple.state, stateActionTuple.action, newValue);
    }

    public float putStabilizer(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.putStabilizing(this, stateActionTuple.state, stateActionTuple.action, newValue);
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        return DynamicValueFunctionTable.put(this, stateActionTuple.state, stateActionTuple.action, newValue);
    }

    public static boolean equivalentStateLander(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = state_a.altitude == state_b.altitude &&
                state_a.velocity == state_b.velocity &&
                state_a.time == state_b.time;
        return equivalent;
    }

    public static boolean equivalentStateStabilizer(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent =
                state_a.positionX == state_b.positionX && state_a.positionY == state_b.positionY &&
                state_a.thrust == state_b.thrust &&
                state_a.angleX == state_b.angleX && state_a.angleZ == state_b.angleZ;
        return equivalent;
    }

    public static boolean equivalentState(State state_a, State state_b) {
        if (state_a == null || state_b == null) return false;
        boolean equivalent = state_a.equals(state_b);
        return equivalent;
    }


    public boolean containsKey(State state, Action action) {
        return true;
        // return checkBounds(new StateActionTuple(state, action));
    }

    public boolean containsKey(StateActionTuple stateActionTuple) {
        return true;
        // return checkBounds(stateActionTuple);
    }

    public boolean checkBounds(State state) {
        return checkBounds(new StateActionTuple(state, new Action(0, 0, 0)));
    }

    public boolean checkBounds(StateActionTuple stateActionTuple) {
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        for (String stateField: ModelBaseImplementation.stateDefinition) {
            int currentValue = state.getField(stateField);
            if ((currentValue < getMinField(stateField)) || (currentValue > getMaxField(stateField))) return false;
        }
        for (String actionField: ModelBaseImplementation.actionDefinition) {
            int currentValue = action.getField(actionField);
            if ((currentValue < getMinField(actionField)) || (currentValue > getMaxField(actionField))) return false;
        }
        return true;
    }

    public int getMinField(String baseField) {
        return boundaryMapMin.get(baseField);
    }

    public int getMaxField(String baseField) {
        return boundaryMapMax.get(baseField);
    }

    public TerminationBooleanTuple alterTerminalStateIfFailure(State state) {
        boolean verticalSuccess = true;
        boolean angleSuccess = true;

        if (state == null) return new TerminationBooleanTuple(true, true);
        for (String landingField: ModelBaseImplementation.stateDefinitionLanding) {
            int minValue = getMinField(landingField);
            int maxValue = getMaxField(landingField);
            int currentValue = state.getField(landingField);
            if (currentValue < minValue) { verticalSuccess = false; state.setField(landingField, minValue); }
            if (currentValue > maxValue) { verticalSuccess = false; state.setField(landingField, maxValue); }
        }
        for (String stabilizingField: ModelBaseImplementation.stateDefinitionStabilizing) {
            int minValue = getMinField(stabilizingField);
            int maxValue = getMaxField(stabilizingField);
            int currentValue = state.getField(stabilizingField);
            if (currentValue < minValue) { angleSuccess = false; state.setField(stabilizingField, minValue); }
            if (currentValue > maxValue) { angleSuccess = false; state.setField(stabilizingField, maxValue); }
        }
        return new TerminationBooleanTuple(verticalSuccess, angleSuccess);
    }

    private float[][][][][][][][][][] allocateNewValueFunctionTable() {
        int[] indeces = generateIndex(ModelBaseImplementation.stateDefinition, ModelBaseImplementation.actionDefinition);
        System.out.println("Allocating stateSpace: " + indexProduct(indeces));
        return (float[][][][][][][][][][]) DynamicValueFunctionTable.callAllocation(indeces);
    }

    private float[][][][] allocateNewLanderValueFunctionTable() {
        int[] indeces = generateIndex(ModelBaseImplementation.stateDefinitionLanding, ModelBaseImplementation.actionDefinitionLanding);
        System.out.println("Allocating stateSpace: " + indexProduct(indeces));
        return (float[][][][]) DynamicValueFunctionTable.callAllocation(indeces);
    }

    private float[][][][][][][] allocateNewStabilizerValueFunctionTable() {
        int[] indeces = generateIndex(ModelBaseImplementation.stateDefinitionStabilizing, ModelBaseImplementation.actionDefinitionStabilizing);
        System.out.println("Allocating stateSpace: " + indexProduct(indeces));
        return (float[][][][][][][]) DynamicValueFunctionTable.callAllocation(indeces);
    }

    private int[] generateIndex(ArrayList<String> stateDefinition, ArrayList<String> actionDefinition){
        int[] indeces = new int[stateDefinition.size() + actionDefinition.size()];
        int index = 0;
        for (String stateField: stateDefinition) {
            indeces[index] = (getMaxField(stateField) - getMinField(stateField) + 1);
            index += 1;
        }
        for (String actionField: actionDefinition) {
            indeces[index] = (getMaxField(actionField) - getMinField(actionField) + 1);
            index += 1;
        }
        return indeces;
    }

    private int indexProduct(int[] indeces) {
        int totalSize = 1;
        for (int index: indeces) {
            totalSize *= index;
        }
        return totalSize;
    }

    public float[][][][][][][][][][] getValueFunctionArray() {
        return valueFunctionTable;
    }

    public float[][][][] getLanderValueFunctionArray() {
        return landerValueFunctionTable;
    }

    public float[][][][][][][] getStabilizerValueFunctionArray() {
        return stabilizerValueFunctionTable;
    }

    public static Action combineCoupledActions(Action landerAction, Action stabilizerAction) {
        return new Action(landerAction.getThrustDouble(), stabilizerAction.getGimbleYDouble(), stabilizerAction.getGimbleZDouble());
    }
}
