package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;

import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.ALTITUDE_PRECISION;

public class TD0 extends ModelBaseImplementation implements ModelInterface {
    public float getExplorationPercentage() { return 0.05f; }
    public void updateStepCommon(ArrayList<StateActionTuple> stateActionTuples,
         Function<StateActionTuple, Float> valueFunction,
         BiFunction<StateActionTuple, Float, Float> putFunction,
         Function<StateActionTuple.State, Float> reward
    ) {
        if(stateActionTuples.size() <= 2) { return; }

        StateActionTuple old = stateActionTuples.get(stateActionTuples.size() - 2);
        StateActionTuple current = stateActionTuples.get(stateActionTuples.size() - 1);

        if (!valueFunctionTable.containsKey(old)) putFunction.apply(old, 0.0f);
        if (!valueFunctionTable.containsKey(current)) putFunction.apply(current, 0.0f);

        float oldValue = valueFunction.apply(old);
        float currentValue = valueFunction.apply(current);
        float rewardValue = reward.apply(current.state);

        putFunction.apply(old,
                oldValue +  alpha * rewardValue + stepDiscount * currentValue - oldValue);
    }

    public void updateTerminalCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<StateActionTuple.State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<StateActionTuple.State, Float> reward
    ) {
        /*
        if (stateActionTuples.get(stateActionTuples.size() - 1).state.getAltitudeDouble() <= ALTITUDE_PRECISION) {
            // positively reward the system
            ModelBaseImplementation mc = new MonteCarlo();
            mc.setValueFunctionTable(this.valueFunctionTable);
            mc.setAlpha(0.1f);
            mc.setTerminalDiscount(0.999f);
            mc.updateTerminalCommon(stateActionTuples, terminalReward, valueFunction, putFunction, reward);
        }
        */
    }

    public float terminalReward(StateActionTuple.State lastState) { return 0.0f; }
    public float reward(StateActionTuple.State state) { return 0.0f; }

    public float terminalLandingReward(StateActionTuple.State lastState) { return 0.0f; }
    public float rewardLanding(StateActionTuple.State state) { return 0.0f; }

    public float terminalStabilizingReward(StateActionTuple.State lastState) {
        // max 100 min 100/31=30  // note all positive!!
        // return 1000.0f * 1.0f / (Math.abs(rewardStabilizing(lastState)) + 1.0f);
        System.out.println("NOT CALLED.");
        return 100.0f * rewardStabilizing(lastState);
    }

    public float rewardStabilizing(StateActionTuple.State state) {
        // max 1 min 1/31  // note all positive
        double angle_in_degrees = state.getAngleZDouble() * (180.0f / Math.PI);
        return 1.0f / (float) (angle_in_degrees * angle_in_degrees + 1.0f);
        // return - (float) (state.getAngleZDouble() * (180.0f / Math.PI));
    }
}
