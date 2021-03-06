package net.sf.openrocket.simulation.extension.impl.rocketlander.methods;

import net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.RLValueFunction;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Function;

public class MonteCarlo extends BaseMethodImplementation {
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
        int[] indeces = valueFunctionManager.getIndecesAndLockAll(SA, lockedIndeces);

        final RLValueFunction valueFunction = lastStateActionTuple.state.definition.valueFunction;
        float[] values = new float[SA.size()];
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = SA.get(timeStep);
            float originalValue = valueFunction.get(indeces[timeStep]);
            values[timeStep] = originalValue + alpha * (G - originalValue);
            G = (terminalDiscount * G) + reward.apply(stateActionTuple.state);
        }

        // thread-safe unlock
        valueFunctionManager.setValueAtIndecesAndUnlockAll(
                lastStateActionTuple.state.definition,
                lockedIndeces,
                indeces,
                values
        );
    }

    public float terminalReward(State lastState) { return 0.0f; }
    public float reward(State state) { return 0.0f; }
}
