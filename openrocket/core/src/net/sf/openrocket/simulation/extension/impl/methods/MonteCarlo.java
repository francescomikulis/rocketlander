package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;

public class MonteCarlo extends ModelBaseImplementation implements ModelInterface {
    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }
    public void updateStepFunction(ArrayList<StateActionTuple> stateActionTuples) {}
    public void updateTerminalFunction(ArrayList<StateActionTuple> stateActionTuples) {}

    public float terminalLandingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardLanding(StateActionTuple.State state) { return 0.0f; }
    public void updateStepLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {}
    public void updateTerminalLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {}

    public float terminalStabilizingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardStabilizing(StateActionTuple.State state) { return 0.0f; }
    public void updateStepStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {}
    public void updateTerminalStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {}
}
