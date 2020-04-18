package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiFunction;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class TD0 extends ModelBaseImplementation implements ModelInterface {
    public TD0 (HashMap<String, HashMap> definition) {
        this.definition = definition;
    }
    public float getExplorationPercentage() { return 0.05f; }
    public void updateStepCommon(ArrayList<StateActionTuple> SA,
         Function<StateActionTuple.State, Float> reward
    ) {
        if(SA.size() <= 2) { return; }

        StateActionTuple old = SA.get(SA.size() - 2);
        StateActionTuple current = SA.get(SA.size() - 1);

        // skip if the states are equivalent under the equivalentStateFunction
        if (OptimizedMap.equivalentState(old.state, current.state)) return;

        if (!valueFunctionTable.containsKey(old)) valueFunctionTable.put(old, 0.0f);
        if (!valueFunctionTable.containsKey(current)) valueFunctionTable.put(current, 0.0f);

        float oldValue = valueFunction(old);
        float currentValue = valueFunction(current);
        float rewardValue = reward.apply(current.state);

        valueFunctionTable.put(old,
                oldValue +  alpha * (rewardValue + stepDiscount * currentValue - oldValue)
        );
    }

    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }

    public float terminalLanderReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardLander(StateActionTuple.State state) { return 0.0f; }

    public float terminalStabilizerReward(StateActionTuple.State lastState) {
        // max 100 min 100/31=30  // note all positive!!
        // return 1000.0f * 1.0f / (Math.abs(rewardStabilizer(lastState)) + 1.0f);
        System.out.println("NOT CALLED.");
        return 100.0f * rewardStabilizer(lastState);
    }

    public float rewardStabilizer(StateActionTuple.State state) {
        float rewardValue = -(float)Math.abs(state.getDouble("angle") * (180.0f / Math.PI));
        // force positive reward for gimbal angle that helps stabilize
        if (Math.signum(state.getDouble("angle")) != Math.signum(state.getDouble("gimbal"))) {
            rewardValue = Math.abs(rewardValue);
        }
        return rewardValue;
        /*
        float realAngleX = (float)(Math.asin(state.getDouble("angleX")) * (180.0f / Math.PI));
        float realAngleY = (float)(Math.asin(state.getDouble("angleY")) * (180.0f / Math.PI));
        return rewardAngleStabilizer(realAngleX) + rewardAngleStabilizer(realAngleY);
         */
    }

    public float terminalReachingReward(StateActionTuple.State lastState) {
        // max 100 min 100/31=30  // note all positive!!
        // return 1000.0f * 1.0f / (Math.abs(rewardStabilizer(lastState)) + 1.0f);
        System.out.println("NOT CALLED.");
        return 100.0f * rewardReaching(lastState);
    }

    public float rewardReaching(StateActionTuple.State state) {
        //float positionReward = rewardPositionStabilizer(state.getDouble("position"), state.getDouble("positionY"));
        float position = (float)Math.abs(state.getDouble("position"));
        float velocity = (float)Math.abs(state.getDouble("velocity"));
        return - position - (velocity/10.0f) / position;

        // float positionReward = -(float)(Math.abs(state.getDouble("position" + state.symmetry)));
        // return positionReward;
    }

    private float rewardPositionStabilizer(double positionX, double positionY){
        positionX = Math.abs(positionX);
        positionY = Math.abs(positionY);
        return -(float)(positionX + positionY);
    }
}
