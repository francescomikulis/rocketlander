package net.sf.openrocket.simulation.extension.impl.methods;

import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;

public interface ModelInterface {
    void updateStepCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> reward
    );

    void updateTerminalCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> terminalReward,
            Function<State, Float> reward
    );

    float terminalReward(State lastState);
    float reward(State state);
}
