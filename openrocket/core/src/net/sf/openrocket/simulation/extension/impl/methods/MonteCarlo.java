package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.BiFunction;

public class MonteCarlo extends ModelBaseImplementation {
    public float getExplorationPercentage() { return 0.05f; }
    public void updateTerminalCommon(
            ArrayList<StateActionTuple> stateActionTuples,
            Function<State, Float> terminalReward,
            Function<StateActionTuple, Float> valueFunction,
            BiFunction<StateActionTuple, Float, Float> putFunction,
            Function<State, Float> reward
    ) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalReward.apply(lastStateActionTuple.state);
        int numExplorationSteps = 0;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = valueFunction.apply(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            putFunction.apply(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (terminalDiscount * G) + reward.apply(stateActionTuple.state);
        }
        System.out.println("Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
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
        float altitude = (float)lastState.getAltitudeDouble();

        return - (landingVelocity * (altitude + 1.0f));
    }
    public float rewardLanding(StateActionTuple.State state) {
        return 0.0f;
    }

    /** Stabilizing **/
    public float terminalStabilizingReward(StateActionTuple.State lastState) {
        return 200.0f * -(float)(Math.abs(lastState.getAngleZDouble()) * (180.0f / Math.PI));
    }
    public float rewardStabilizing(StateActionTuple.State state) {
        return -(float) (Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI));
    }
}
