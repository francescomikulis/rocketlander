package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;

public class MonteCarlo extends ModelBaseImplementation {
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
    public void updateStepFunction(ArrayList<StateActionTuple> stateActionTuples) {
        // monte carlo does not do step updates
    }
    public void updateTerminalFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            try {
                float originalValue = valueFunction(stateActionTuple);
                if (originalValue == 0.0f) numExplorationSteps++;
                valueFunctionTable.put(stateActionTuple, originalValue + alpha * (G - originalValue));
                G = (discount * G) - reward(stateActionTuple.state);
            } catch (Exception e) {
                System.out.println("EXCEPTION HERE WHEN EVALUATING VALUE FUNCTION");
            }
        }
        System.out.println("Combined - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
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
    public void updateStepLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        // monte carlo does not do step updates
    }
    public void updateTerminalLandingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalLandingReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = landingValueFunction(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            valueFunctionTable.putLander(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardLanding(stateActionTuple.state);
        }
        System.out.println("Landing - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
    }

    /** Stabilizing **/
    public float terminalStabilizingReward(StateActionTuple.State lastState) {
        return 200.0f * -(float)(Math.abs(lastState.getAngleZDouble()) * (180.0f / Math.PI));
    }
    public float rewardStabilizing(StateActionTuple.State state) {
        // range: [1/(max Z); 1]
        return (float) (1.0 / (Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI) + 1.0));
    }
    public void updateStepStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        // monte carlo does not do step updates
    }
    public void updateTerminalStabilizingFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalStabilizingReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = stabilizingValueFunction(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            valueFunctionTable.putStabilizer(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardStabilizing(stateActionTuple.state);
        }
        System.out.println("Stabilizing - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
    }
}
