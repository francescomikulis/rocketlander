package net.sf.openrocket.simulation.extension.impl.rocketlander.methods;

import net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;

import java.util.ArrayList;
import java.util.function.Function;


public class TD0 extends BaseMethodImplementation implements MethodInterface {
    public TD0 (MDPDefinition definition) {
        this.definition = definition;
    }
    public void updateStepCommon(ArrayList<StateActionTuple> SA,
         Function<StateActionTuple.State, Float> reward
    ) {
        if(SA == null) { return; }
        if(SA.size() <= 2) { return; }

        StateActionTuple old = SA.get(SA.size() - 2);
        StateActionTuple current = SA.get(SA.size() - 1);

        float currentValue = valueFunction(current);
        float rewardValue = reward.apply(current.state);

        // thread safety
        int index = valueFunctionManager.getIndexAndLock(old);
        float oldValue = old.state.definition.valueFunction.get(index);
        float newValue = oldValue +  alpha * (rewardValue + stepDiscount * currentValue - oldValue);
        valueFunctionManager.setValueAtIndexAndUnlock(old, index, newValue);
    }

    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }
}
