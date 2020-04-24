package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.function.BiFunction;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class TD0 extends ModelBaseImplementation implements ModelInterface {
    public TD0 (HashMap<String, LinkedHashMap> definition) {
        this.definition = definition;
    }
    public float getExplorationPercentage() { return 0.03f; }
    public void updateStepCommon(ArrayList<StateActionTuple> SA,
         Function<StateActionTuple.State, Float> reward
    ) {
        if(SA == null) { return; }
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
        if (Float.isNaN(rewardValue))
            System.out.println("NAN REWARD!");

        float newValue = oldValue +  alpha * (rewardValue + stepDiscount * currentValue - oldValue);
        if (Float.isNaN(newValue))
            System.out.println("NAN UPDATE!");

        valueFunctionTable.put(old, newValue);
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
        float angleInDegrees = (float)Math.abs(state.getDouble("angle") * (180.0f / Math.PI));
        float angleVelocityInDegrees = (float)Math.abs(state.getDouble("angleVelocity") * (180.0f / Math.PI));
        // return - (10 * (angleInDegrees * angleInDegrees) + (angleVelocityInDegrees * angleVelocityInDegrees)) + 100;
        return - (angleInDegrees * angleInDegrees) + 1;

        //float rewardValue = -(float)Math.abs(state.getDouble("angle") * (180.0f / Math.PI));

        // force positive reward for gimbal angle that helps stabilize
        /*
        if (Math.signum(state.getDouble("angle")) != Math.signum(state.getDouble("gimbal"))) {
            rewardValue = Math.abs(rewardValue);
        }

        return rewardValue;
         */

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

        float reward;
        if (position != 0.0) {
            reward = - position * 10 - velocity;  // velocity / position
        } else {
            reward = - position * 10 - velocity;
        }
        if (Math.signum(state.getDouble("position")) != Math.signum(state.getDouble("gimbal"))) {
            reward = reward + 30;
        }

        return reward;

        // float positionReward = -(float)(Math.abs(state.getDouble("position" + state.symmetry)));
        // return positionReward;
    }

    private float rewardPositionStabilizer(double positionX, double positionY){
        positionX = Math.abs(positionX);
        positionY = Math.abs(positionY);
        return -(float)(positionX + positionY);
    }
}
