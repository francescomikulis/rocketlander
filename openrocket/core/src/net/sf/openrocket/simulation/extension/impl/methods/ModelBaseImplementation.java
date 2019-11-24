package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class ModelBaseImplementation {
    OptimizedMap valueFunctionTable = null;

    float valueFunction(State state, Action action) { return valueFunction(new StateActionTuple(state, action)); }
    float valueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.get(stateActionTuple);
    }

    float landingValueFunction(State state, Action action) { return landingValueFunction(new StateActionTuple(state, action)); }
    float landingValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getLander(stateActionTuple);
    }
    float stabilizingValueFunction(State state, Action action) { return stabilizingValueFunction(new StateActionTuple(state, action)); }
    float stabilizingValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getStabilizer(stateActionTuple);
    }
}
