package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.function.Function;
import java.util.function.BiFunction;

public class MonteCarlo extends ModelBaseImplementation {
    public MonteCarlo (MDPDefinition definition) {
        this.definition = definition;
    }
    public void updateTerminalCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> terminalReward,
            Function<State, Float> reward
    ) {
        if(SA == null) { return; }
        if(SA.size() == 0) { return; }

        int lastTimeStep = SA.size() - 1;
        StateActionTuple lastStateActionTuple = SA.get(lastTimeStep);

        double positionPenalty = 0;
        double positionZ = Math.abs(lastStateActionTuple.state.getDouble("positionZ"));
        if (positionZ > 0.5)
            positionPenalty = 10000.0 * positionZ;
        float G = terminalReward.apply(lastStateActionTuple.state) - (float)positionPenalty;

        // thread-safe lock
        HashSet<Integer> lockedIndeces = new HashSet<>();
        int[] indeces = valueFunctionTable.getIndecesAndLockAll(SA, lockedIndeces);

        final float[] valueFunction = lastStateActionTuple.state.definition.valueFunction;
        float[] values = new float[SA.size()];
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = SA.get(timeStep);
            float originalValue = valueFunction[indeces[timeStep]];
            values[timeStep] = originalValue + alpha * (G - originalValue);
            G = (terminalDiscount * G) + reward.apply(stateActionTuple.state);
        }

        // thread-safe unlock
        valueFunctionTable.setValueAtIndecesAndUnlockAll(
                lastStateActionTuple.state.definition,
                lockedIndeces,
                indeces,
                values
        );
    }

    public float terminalReward(State lastState) { return 0.0f; }
    public float reward(State state) { return 0.0f; }
}
