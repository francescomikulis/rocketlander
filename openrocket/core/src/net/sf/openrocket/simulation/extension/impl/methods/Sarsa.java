package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

public class Sarsa extends ModelBaseImplementation implements ModelInterface {
    public float getExplorationPercentage() { return 0.05f; }
    public void updateStepCommon(
        ArrayList<StateActionTuple> stateActionTuples,
        Function<StateActionTuple, Float> valueFunction,
        BiFunction<StateActionTuple, Float, Float> putFunction,
        Function<StateActionTuple.State, Float> reward
    ) {
        double[] weight = new double[]{1.0,1.0};
        double discount = 0.999;
        double alpha = 0.3;

        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple currentStateAction = stateActionTuples.get(lastTimeStep);
        double rew = reward(currentStateAction.state);

        double[] evaluation = new double[]{0.0,0.0};
        evaluation[0] = discount * (weight[0] * currentStateAction.action.thrust);
        evaluation[1] = discount * (weight[1] * currentStateAction.state.velocity);


        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep - 1);

        double[] previous = new double[]{0.0,0.0};
        previous[0] = weight[0] * lastStateActionTuple.action.thrust;
        previous[1] = weight[1] * lastStateActionTuple.state.velocity;

        double[] gradient = new double[]{1.0,1.0};
        gradient[0] = lastStateActionTuple.action.thrust;
        gradient[1] = lastStateActionTuple.state.velocity;

        weight[0] = weight[0] + alpha * (rew + evaluation[0] - previous[0]) * gradient[0];
        weight[1] = weight[1] + alpha * (rew + evaluation[1] - previous[1]) * gradient[1];

        //return valueFunction(episodeStateActions, stateActionTuple);
        //return 0.0f;
    }

    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }

    public float terminalLandingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardLanding(StateActionTuple.State state) { return 0.0f; }

    public float terminalStabilizingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardStabilizing(StateActionTuple.State state) { return 0.0f; }
}
