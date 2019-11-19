package net.sf.openrocket.simulation.extension.impl;

import java.lang.reflect.Type;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

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
    private float[][][][][][][] valueFunctionTable = null;
    private int minAltitude, minVelocity, minThrust, minAngleX, minAngleZ, minGimbleY, minGimbleZ;
    private int maxAltitude, maxVelocity, maxThrust, maxAngleX, maxAngleZ, maxGimbleY, maxGimbleZ;

    public OptimizedMap() {
        constructorCode(null);
    }

    public OptimizedMap(float[][][][][][][] valueFunctionTable) {
        constructorCode(valueFunctionTable);
    }

    public void constructorCode(float[][][][][][][] newValueFunctionTable) {
        // generate low minimum values
        State lowState = new State(null);
        minAltitude = lowState.setAltitude(MIN_ALTITUDE).altitude;
        minVelocity = lowState.setVelocity(MIN_VELOCITY).velocity;
        minThrust = lowState.setThrust(MIN_THRUST).thrust;
        minAngleX = lowState.setAngleX(- Math.PI).angleX;
        minAngleZ = lowState.setAngleZ(MIN_TERMINAL_ORIENTATION_Z).angleZ;
        minGimbleY = lowState.setGimbleY(- Math.PI).gimbleY;
        minGimbleZ = lowState.setGimbleZ(MIN_GIMBLE_Z).gimbleZ;
        // generate high maximum values
        State highState = new State(null);
        maxAltitude = highState.setAltitude(MAX_ALTITUDE).altitude;
        maxVelocity = highState.setVelocity(MAX_VELOCITY).velocity;
        maxThrust = highState.setThrust(MAX_THRUST).thrust;
        maxAngleX = highState.setAngleX(1 * Math.PI).angleX;
        maxAngleZ = highState.setAngleZ(MAX_TERMINAL_ORIENTATION_Z).angleZ;
        maxGimbleY = highState.setGimbleY(1 * Math.PI).gimbleY;
        maxGimbleZ = highState.setGimbleZ(MAX_GIMBLE_Z).gimbleZ;
        // allocate new function table
        if (newValueFunctionTable == null)
            newValueFunctionTable = allocateNewValueFunctionTable();
        this.valueFunctionTable = newValueFunctionTable;
    }

    public float get(StateActionTuple stateActionTuple) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        return valueFunctionTable
                [state.altitude - minAltitude][state.velocity - minVelocity][state.angleX - minAngleX][state.angleZ - minAngleZ]
                [action.thrust - minThrust][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ];
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        valueFunctionTable
                [state.altitude - minAltitude][state.velocity - minVelocity][state.angleX - minAngleX][state.angleZ - minAngleZ]
                [action.thrust - minThrust][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ] = newValue;
        return newValue;
    }

    public boolean containsKey(StateActionTuple stateActionTuple) {
        return checkBounds(stateActionTuple);
    }

    public boolean checkBounds(State state) {
        return checkBounds(new StateActionTuple(state, new Action(0, 0, 0)));
    }

    public boolean checkBounds(StateActionTuple stateActionTuple) {
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;

        // stateCheck
        if ((state.angleZ < minAngleZ) || (state.angleZ > maxAngleZ)) return false;
        if ((state.altitude < minAltitude) || (state.altitude > maxAltitude)) return false;
        if ((state.velocity < minVelocity) || (state.velocity > maxVelocity)) return false;
        if ((state.angleX < minAngleX) || (state.angleX > maxAngleX)) return false;
        // actionCheck
        if ((action.thrust < minThrust) || (action.thrust > maxThrust)) return false;
        if ((action.gimbleY < minGimbleY) || (action.gimbleY > maxGimbleY)) return false;
        if ((action.gimbleZ < minGimbleZ) || (action.gimbleZ > maxGimbleZ)) return false;

        return true;
    }


    private float[][][][][][][] allocateNewValueFunctionTable() {
        int altitudeSize = maxAltitude - minAltitude + 1;
        int velocitySize = maxVelocity - minVelocity + 1;
        int thrustSize = maxThrust - minThrust + 1;
        int angleXSize = maxAngleX - minAngleX + 1;
        int angleZSize = maxAngleZ - minAngleZ + 1;
        int gimbleYSize = maxGimbleY - minGimbleY + 1;
        int gimbleZSize = maxGimbleZ - minGimbleZ + 1;
        double stateSpace = altitudeSize * velocitySize * angleXSize * angleZSize * thrustSize * gimbleYSize * gimbleZSize;
        System.out.println("SAMPLE EXAMPLE 1 HAHAHA: " + stateSpace);
        System.out.println("SAMPLE EXAMPLE 2 HAHAHA: " + stateSpace);
        System.out.println("SAMPLE EXAMPLE 3 HAHAHA: " + stateSpace);
        return new float
                [altitudeSize][velocitySize][angleXSize][angleZSize]
                [thrustSize][gimbleYSize][gimbleZSize];
    }

    public float[][][][][][][] getValueFunctionArray() {
        return valueFunctionTable;
    }
}
