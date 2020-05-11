package net.sf.openrocket.simulation.extension.impl.rocketlander.methods;

import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.State;

public interface MethodInterface {
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
