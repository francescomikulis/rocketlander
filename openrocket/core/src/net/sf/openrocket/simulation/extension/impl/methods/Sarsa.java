package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

public class Sarsa extends ModelBaseImplementation implements ModelInterface {
    public float getExplorationPercentage() { return 0.05f; }
    public void updateStepCommon(
        ArrayList<StateActionTuple> SA,
        Function<StateActionTuple, Float> valueFunction,
        BiFunction<StateActionTuple, Float, Float> putFunction,
        Function<StateActionTuple.State, Float> reward
    ) {
        double[] weight = new double[]{1.0,1.0};
        double discount = 0.999;
        double alpha = 0.3;

        int lastTimeStep = SA.size() - 1;
        StateActionTuple currentStateAction = SA.get(lastTimeStep);
        double rew = reward(currentStateAction.state);

        double[] evaluation = new double[]{0.0,0.0};
        evaluation[0] = discount * (weight[0] * currentStateAction.action.get("thrust"));
        evaluation[1] = discount * (weight[1] * currentStateAction.state.get("velocity"));


        StateActionTuple lastStateActionTuple = SA.get(lastTimeStep - 1);

        double[] previous = new double[]{0.0,0.0};
        previous[0] = weight[0] * lastStateActionTuple.action.get("thrust");
        previous[1] = weight[1] * lastStateActionTuple.state.get("velocity");

        double[] gradient = new double[]{1.0,1.0};
        gradient[0] = lastStateActionTuple.action.get("thrust");
        gradient[1] = lastStateActionTuple.state.get("velocity");

        weight[0] = weight[0] + alpha * (rew + evaluation[0] - previous[0]) * gradient[0];
        weight[1] = weight[1] + alpha * (rew + evaluation[1] - previous[1]) * gradient[1];

        //return valueFunction(episodeStateActions, stateActionTuple);
        //return 0.0f;
    }

    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }

    public float terminalLanderReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardLander(StateActionTuple.State state) { return 0.0f; }

    public float terminalStabilizerReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardStabilizer(StateActionTuple.State state) { return 0.0f; }

    public float terminalReachingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardReaching(StateActionTuple.State state) { return 0.0f; }
}
