package net.sf.openrocket.simulation.extension.impl.methods;

import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.OptimizedMap;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.function.Function;
import java.util.function.BiFunction;

public class MonteCarlo extends ModelBaseImplementation {
    public MonteCarlo (MDPDefinition definition) {
        this.definition = definition;
    }
    public void updateTerminalCommon(
            ArrayList<StateActionTuple> SA,
            Function<State, Float> terminalReward,
            Function<State, Float> reward
    ) {
        if(SA == null) { return; }
        if(SA.size() == 0) { return; }

        int lastTimeStep = SA.size() - 1;
        StateActionTuple lastStateActionTuple = SA.get(lastTimeStep);
        int actualSteps = 0;

        double positionPenalty = 0;
        double positionZ = Math.abs(lastStateActionTuple.state.getDouble("positionZ"));
        if (positionZ > 0.5)
            positionPenalty = 10000.0 * positionZ;
        float G = terminalReward.apply(lastStateActionTuple.state) - (float)positionPenalty;
        int numExplorationSteps = 0;

        // thread-safe lock
        HashSet<Integer> lockedIndeces = new HashSet<>();
        int[] indeces = valueFunctionTable.getIndecesAndLockAll(SA, lockedIndeces);

        final float[] valueFunction = lastStateActionTuple.state.definition.valueFunction;
        float[] values = new float[SA.size()];
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = SA.get(timeStep);

            actualSteps += 1;
            float originalValue = valueFunction[indeces[timeStep]];
            if (originalValue == 0.0f) numExplorationSteps++;
            values[timeStep] = originalValue + alpha * (G - originalValue);
            G = (terminalDiscount * G) + reward.apply(stateActionTuple.state);
        }

        // thread-safe unlock
        valueFunctionTable.setValueAtIndecesAndUnlockAll(
                lastStateActionTuple.state.definition,
                lockedIndeces,
                indeces,
                values
        );

        // System.out.println("Exploration ratio " + (numExplorationSteps * 100.0f)/actualSteps + "% out of " + actualSteps + " states");
    }

    private int getActualIntPositionZ(State state) {
        int minIntField = Integer.MAX_VALUE;
        for (String field: state.definition.stateDefinitionFields) {
            if (field.toLowerCase().contains("positionz")) {
                minIntField = Math.min(minIntField, state.get(field));
            }
        }
        return minIntField;
    }

    /** Traditional Implementation **/
    public float terminalReward(State lastState) {
        float realAngleX = (float)(Math.abs(Math.asin(lastState.getDouble("angleX")) * (180.0f / Math.PI)));
        float realAngleY = (float)(Math.abs(Math.asin(lastState.getDouble("angleY")) * (180.0f / Math.PI)));
        // float angleFromZ = (float)(Math.abs(lastState.getDouble("angleZ")) * (180.0f / Math.PI));
        float landerVelocity = (float)Math.abs(lastState.getDouble("velocityZ"));
        float altitude = (float)Math.abs(lastState.getDouble("positionZ"));
        return -(realAngleX + realAngleY + landerVelocity) * (altitude + 1.0f);
    }
    public float reward(State state) {
        // return -(float)(Math.abs(state.getDouble("angleZ")) * (180.0f / Math.PI));
        float realAngleX = (float)(Math.abs(state.getDouble("angleX") * (180.0f / Math.PI)));
        float realAngleY = (float)(Math.abs(state.getDouble("angleY") * (180.0f / Math.PI)));
        return -(realAngleX + realAngleY);
    }

    /** Coupled Implementation **/

    /** Lander **/
    public float terminalLanderReward(State lastState) {
        float landerVelocity = (float)Math.abs(lastState.getDouble("velocityZ"));
        return - (landerVelocity);  // +(float)lastState.getDouble("time")
    }
    public float rewardLander(StateActionTuple.State state) {
        return - (float)(state.getDouble("thrust") / 100.0f);
    }

    /** Stabilizer **/
    public float terminalStabilizerReward(StateActionTuple.State lastState) {
        return 10.0f * rewardStabilizer(lastState);
    }
    public float rewardStabilizer(StateActionTuple.State state) {
        // return -(float) (Math.abs(state.getDouble("angleZ") * (180.0f / Math.PI)));
        return -(float)(Math.abs(state.getDouble("angle") * (180.0f / Math.PI)));
    }

    /** Reaching **/
    public float terminalReachingReward(StateActionTuple.State lastState) {
        return 10.0f * rewardReaching(lastState);
    }
    public float rewardReaching(StateActionTuple.State state) {
        float position = (float)Math.abs(state.getDouble("position"));
        // return -(float)(Math.abs(Math.asin(state.getDouble("angle")) * (180.0f / Math.PI)));
        return -position;
    }
}
