package net.sf.openrocket.simulation.extension.impl.methods;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;

public interface ModelInterface {
    float getExplorationPercentage();
    void updateStepCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward,
            BiFunction<State, State, Boolean> equivalentState
    );

    void updateTerminalCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward,
            BiFunction<State, State, Boolean> equivalentState
    );

    float terminalReward(State lastState);
    float reward(State state);
    void updateStepFunction(ArrayList<StateActionTuple> stateActionTuples);
    void updateTerminalFunction(ArrayList<StateActionTuple> stateActionTuples);

    float terminalLandingReward(State lastState);
    float rewardLanding(State state);
    void updateStepLandingFunction(ArrayList<StateActionTuple> stateActionTuples);
    void updateTerminalLandingFunction(ArrayList<StateActionTuple> stateActionTuples);

    float terminalStabilizingReward(State lastState);
    float rewardStabilizing(State state);
    void updateStepStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples);
    void updateTerminalStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples);
}
