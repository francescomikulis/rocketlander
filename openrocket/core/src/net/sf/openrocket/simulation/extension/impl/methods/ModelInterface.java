package net.sf.openrocket.simulation.extension.impl.methods;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;

public interface ModelInterface {
    float getExplorationPercentage();
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
    void updateStepFunction(ArrayList<StateActionTuple> SA);
    void updateTerminalFunction(ArrayList<StateActionTuple> SA);

    float terminalLanderReward(State lastState);
    float rewardLander(State state);
    void updateStepLanderFunction(ArrayList<StateActionTuple> SA);
    void updateTerminalLanderFunction(ArrayList<StateActionTuple> SA);

    float terminalStabilizerReward(State lastState);
    float rewardStabilizer(State state);
    void updateStepStabilizerFunction(ArrayList<StateActionTuple> SA);
    void updateTerminalStabilizerFunction(ArrayList<StateActionTuple> SA);

    float terminalReachingReward(State lastState);
    float rewardReaching(State state);
    void updateStepReachingFunction(ArrayList<StateActionTuple> SA);
    void updateTerminalReachingFunction(ArrayList<StateActionTuple> SA);
}
