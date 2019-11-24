package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;

public class TD0 extends ModelBaseImplementation implements ModelInterface {
    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }
    public void updateStepFunction(ArrayList<StateActionTuple> stateActionTuples) {
        if(stateActionTuples.size() <= 2) { return; }

        StateActionTuple old = stateActionTuples.get(stateActionTuples.size() - 2);
        StateActionTuple current = stateActionTuples.get(stateActionTuples.size() - 1);
        float alpha = 0.3f;
        float gamma = 0.999f;

        if (!valueFunctionTable.containsKey(old)) valueFunctionTable.put(old, 0.0f);
        if (!valueFunctionTable.containsKey(current)) valueFunctionTable.put(current, 0.0f);

        valueFunctionTable.put(old, valueFunctionTable.get(old) +
                alpha * (reward(current.state) + gamma * valueFunctionTable.get(current) - valueFunctionTable.get(old)));
    }
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
