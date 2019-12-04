package net.sf.openrocket.simulation.extension.impl;

import java.util.function.Function;

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
    enum MapMethod {
        Traditional, Coupled
    }
    public static MapMethod mapMethod = MapMethod.Coupled;

    private float[][][][][][][][] valueFunctionTable = null;
    private float[][][][] landerValueFunctionTable = null;
    private float[][][][][] stabilizerValueFunctionTable = null;
    private int minAltitude, minVelocity, minTime, minThrust, minAngleX, minAngleZ, minGimbleY, minGimbleZ;
    private int maxAltitude, maxVelocity, maxTime, maxThrust, maxAngleX, maxAngleZ, maxGimbleY, maxGimbleZ;

    public OptimizedMap() {
        if (mapMethod == MapMethod.Traditional)
            constructorTraditional(null);
        else if (mapMethod == MapMethod.Coupled)
            constructorCoupled(null, null);
    }

    public OptimizedMap(float[][][][][][][][] newValueFunctionTable) {
        constructorTraditional(newValueFunctionTable);
    }

    private void constructorTraditional(float[][][][][][][][] newValueFunctionTable) {
        constructorCode();

        // allocate new function table
        if (newValueFunctionTable == null)
            newValueFunctionTable = allocateNewValueFunctionTable();
        this.valueFunctionTable = newValueFunctionTable;
    }

    public OptimizedMap(float[][][][] newLanderValueFunctionTable, float[][][][][] newStabilizerValueFunctionTable) {
        constructorCoupled(newLanderValueFunctionTable, newStabilizerValueFunctionTable);
    }

    private void constructorCoupled(float[][][][] newLanderValueFunctionTable, float[][][][][] newStabilizerValueFunctionTable) {
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
        minVelocity = lowState.setVelocity(MIN_VELOCITY).velocity;
        minTime = lowState.setTime(MIN_TIME).time;
        minThrust = lowState.setThrust(MIN_THRUST).thrust;
        minAngleX = lowState.setAngleX(0).angleX;
        minAngleZ = lowState.setAngleZ(MIN_TERMINAL_ORIENTATION_Z).angleZ;
        minGimbleY = lowState.setGimbleY(0).gimbleY;
        minGimbleZ = lowState.setGimbleZ(MIN_GIMBLE_Z).gimbleZ;
        // generate high maximum values
        State highState = new State(null);
        maxAltitude = highState.setAltitude(MAX_ALTITUDE).altitude;
        maxVelocity = highState.setVelocity(MAX_VELOCITY).velocity;
        maxTime = highState.setTime(MAX_TIME).time;
        maxThrust = highState.setThrust(MAX_THRUST).thrust;
        maxAngleX = highState.setAngleX(2 * MAX_HALF_CIRCLE - 0.00001).angleX;
        maxAngleZ = highState.setAngleZ(MAX_TERMINAL_ORIENTATION_Z).angleZ;
        maxGimbleY = highState.setGimbleY(2 * MAX_HALF_CIRCLE - 0.00001).gimbleY;
        maxGimbleZ = highState.setGimbleZ(MAX_GIMBLE_Z).gimbleZ;
    }

    public float getLander(StateActionTuple stateActionTuple) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        return landerValueFunctionTable[state.altitude - minAltitude][state.velocity - minVelocity][state.time - minTime][action.thrust - minThrust];
    }

    public float getStabilizer(StateActionTuple stateActionTuple) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        return stabilizerValueFunctionTable
                [action.thrust - minThrust][state.angleX - minAngleX][state.angleZ - minAngleZ][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ];
    }

    public float get(StateActionTuple stateActionTuple) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        return valueFunctionTable
                [state.altitude - minAltitude][state.velocity - minVelocity][state.time - minTime][state.angleX - minAngleX][state.angleZ - minAngleZ]
                [action.thrust - minThrust][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ];
    }

    public float putLander(StateActionTuple stateActionTuple, float newValue) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        landerValueFunctionTable
                [state.altitude - minAltitude][state.velocity - minVelocity][state.time - minTime][action.thrust - minThrust] = newValue;
        return newValue;
    }

    public float putStabilizer(StateActionTuple stateActionTuple, float newValue) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        stabilizerValueFunctionTable
                [action.thrust - minThrust][state.angleX - minAngleX]
                [state.angleZ - minAngleZ][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ] = newValue;
        return newValue;
    }

    public float put(StateActionTuple stateActionTuple, float newValue) {
        if (!checkBounds(stateActionTuple)) return Float.NEGATIVE_INFINITY;
        State state = stateActionTuple.state;
        Action action = stateActionTuple.action;
        valueFunctionTable
                [state.altitude - minAltitude][state.velocity - minVelocity][state.time - minTime]
                [state.angleX - minAngleX][state.angleZ - minAngleZ]
                [action.thrust - minThrust][action.gimbleY - minGimbleY][action.gimbleZ - minGimbleZ] = newValue;
        return newValue;
    }

    public boolean containsKey(State state, Action action) {
        return checkBounds(new StateActionTuple(state, action));
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
        if ((state.time < minTime) || (state.time > maxTime)) return false;
        // actionCheck
        if ((action.thrust < minThrust) || (action.thrust > maxThrust)) {
            System.out.println("THIS SHOULD NEVER EVER HAPPEN");
            return false;
        }
        if ((action.gimbleY < minGimbleY) || (action.gimbleY > maxGimbleY)) return false;
        if ((action.gimbleZ < minGimbleZ) || (action.gimbleZ > maxGimbleZ)) return false;

        return true;
    }

    public TerminationBooleanTuple alterTerminalStateIfFailure(State state) {
        boolean verticalSuccess = true;
        boolean angleSuccess = true;

        if (state.altitude < minAltitude) { verticalSuccess = false; state.altitude = minAltitude; }
        if (state.altitude > maxAltitude) { verticalSuccess = false; state.altitude = maxAltitude; }
        if (state.velocity < minVelocity) { verticalSuccess = false; state.velocity = minVelocity; }
        if (state.velocity > maxVelocity) { verticalSuccess = false; state.velocity = maxVelocity; }
        if (state.angleZ < minAngleZ) { angleSuccess = false; state.angleZ = minAngleZ; }
        if (state.angleZ > maxAngleZ) { angleSuccess = false; state.angleZ = maxAngleZ; }
        if (state.angleX < minAngleX) { angleSuccess = false; state.angleX = minAngleX; }
        if (state.angleX > maxAngleX) { angleSuccess = false; state.angleX = maxAngleX; }
        // angle stabilization for the last meter appears impossible.  Force allow success if within rounded range.
        if (state.altitude == minAltitude) { verticalSuccess = true; angleSuccess = true; }

        return new TerminationBooleanTuple(verticalSuccess, angleSuccess);
    }

    private float[][][][][][][][] allocateNewValueFunctionTable() {
        int altitudeSize = maxAltitude - minAltitude + 1;
        int velocitySize = maxVelocity - minVelocity + 1;
        int timeSize = maxTime - minTime + 1;
        int thrustSize = maxThrust - minThrust + 1;
        int angleXSize = maxAngleX - minAngleX + 1;
        int angleZSize = maxAngleZ - minAngleZ + 1;
        int gimbleYSize = maxGimbleY - minGimbleY + 1;
        int gimbleZSize = maxGimbleZ - minGimbleZ + 1;
        double stateSpace = altitudeSize * velocitySize * timeSize * angleXSize * angleZSize * thrustSize * gimbleYSize * gimbleZSize;
        System.out.println("Allocating stateSpace: " + stateSpace);
        return new float
                [altitudeSize][velocitySize][timeSize][angleXSize][angleZSize]
                [thrustSize][gimbleYSize][gimbleZSize];
    }

    private float[][][][] allocateNewLanderValueFunctionTable() {
        int altitudeSize = maxAltitude - minAltitude + 1;
        int velocitySize = maxVelocity - minVelocity + 1;
        int timeSize = maxTime - minTime + 1;
        int thrustSize = maxThrust - minThrust + 1;
        double stateSpace = altitudeSize * velocitySize * timeSize * thrustSize;
        System.out.println("Allocating stateSpace: " + stateSpace);
        return new float[altitudeSize][velocitySize][timeSize][thrustSize];
    }

    private float[][][][][] allocateNewStabilizerValueFunctionTable() {
        int thrustSize = maxThrust - minThrust + 1;
        int angleXSize = maxAngleX - minAngleX + 1;
        int angleZSize = maxAngleZ - minAngleZ + 1;
        int gimbleYSize = maxGimbleY - minGimbleY + 1;
        int gimbleZSize = maxGimbleZ - minGimbleZ + 1;
        double stateSpace = angleXSize * angleZSize * thrustSize * gimbleYSize * gimbleZSize;
        System.out.println("Allocating stateSpace: " + stateSpace);
        return new float[thrustSize][angleXSize][angleZSize][gimbleYSize][gimbleZSize];
    }

    public float[][][][][][][][] getValueFunctionArray() {
        return valueFunctionTable;
    }

    public float[][][][] getLanderValueFunctionArray() {
        return landerValueFunctionTable;
    }

    public float[][][][][] getStabilizerValueFunctionArray() {
        return stabilizerValueFunctionTable;
    }

    public static Action combineCoupledActions(Action landerAction, Action stabilizerAction) {
        return new Action(landerAction.getThrustDouble(), stabilizerAction.getGimbleYDouble(), stabilizerAction.getGimbleZDouble());
    }
}
