package net.sf.openrocket.simulation.extension.impl.methods;

import java.util.*;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;

public interface ModelInterface {
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
