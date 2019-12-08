package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.BiFunction;

public class MonteCarlo extends ModelBaseImplementation {
    public float getExplorationPercentage() { return 0.02f; }
    public void updateTerminalCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward,
            BiFunction<State, State, Boolean> equivalentState
    ) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        int actualSteps = 0;
        float G = terminalReward.apply(lastStateActionTuple.state);
        int numExplorationSteps = 0;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);

            // skip if the states are equivalent under the equivalentStateFunction
            if (equivalentState.apply(lastStateActionTuple.state, stateActionTuple.state)) continue;
            lastStateActionTuple = stateActionTuple;

            actualSteps += 1;
            float originalValue = valueFunction.apply(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            putFunction.apply(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (terminalDiscount * G) + reward.apply(stateActionTuple.state);
        }
        System.out.println("Exploration ratio " + (numExplorationSteps * 100.0f)/actualSteps + "% out of " + actualSteps + " states");
    }

    /** Traditional Implementation **/
    public float terminalReward(State lastState) {
        float angleFromZ = (float)(Math.abs(lastState.getAngleZDouble()) * (180.0f / Math.PI));
        float landingVelocity = (float)Math.abs(lastState.getVelocityDouble());
        float altitude = (float)lastState.getAltitudeDouble();
        return -(angleFromZ + landingVelocity) * (altitude + 1.0f);
    }
    public float reward(State state) {
        return -(float)(Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI));
    }

    /** Coupled Implementation **/

    /** Landing **/
    public float terminalLandingReward(State lastState) {
        float landingVelocity = (float)Math.abs(lastState.getVelocityDouble());
        return - (landingVelocity);
    }
    public float rewardLanding(StateActionTuple.State state) {
        return - (float)(state.getThrustDouble() / 100.0f);
    }

    /** Stabilizing **/
    public float terminalStabilizingReward(StateActionTuple.State lastState) {
        return 10.0f * rewardStabilizing(lastState);
    }
    public float rewardStabilizing(StateActionTuple.State state) {
        return -(float) (Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI));
    }
}
