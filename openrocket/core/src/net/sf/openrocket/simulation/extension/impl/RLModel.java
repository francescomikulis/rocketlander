package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;

    // MonteCarlo or TD0
    private ModelBaseImplementation primaryMethod = new MonteCarlo(landerDefinition);
    private ModelBaseImplementation secondaryMethod = new TD0(reacherDefinition);
    private ModelBaseImplementation tertiaryMethod = new TD0(stabilizerDefinition);
    public SimulationType simulationType = SimulationType._1D;

    enum SimulationType {
        _1D, _2D, _3D
    }

    private static volatile RLModel instance;

    private RLModel(){}

    public static RLModel getInstance() {
        if (instance == null) { // first time lock
            synchronized (RLModel.class) {
                if (instance == null) {  // second time lock
                    instance = new RLModel();
                }
            }
        }
        return instance;
    }


    public void initializeModel() {
        episodeManager = RLEpisodeManager.getInstance();
        episodeManager.safeActionValueFunctionInitialization();
    }

    private HashSet<Double> generatePossibleActionValues(double value, float[] definition) {
        return generatePossibleActionValues(value, definition[0], definition[1], definition[2], definition[1] - definition[0]);
    }

    private HashSet<Double> generatePossibleActionValues(double value, double MIN_VAL, double MAX_VAL, double smallest_increment, double max_increment) {
        HashSet<Double> possibleActionValues = new HashSet<>();

        int number_of_loops = (int) (2.0 * max_increment / smallest_increment) + 1;
        double starting_low_value =  value - max_increment;
        for (int i = 0; i < number_of_loops; i++) {
            double possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL)) {
                possibleActionValues.add(possibleValue);
            }
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public HashSet<Action> generatePossibleActions(State state) {
        return generatePossibleActions(state, new HashSet<>(), new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues) {
        return generatePossibleActions(state, overrideThrustValues, new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> possibleGimbalXValues, HashSet<Double> possibleGimbalYValues) {
        return generatePossibleActions(state, new HashSet<>(), possibleGimbalXValues, possibleGimbalYValues);
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues, HashSet<Double> overrideGimbalXValues, HashSet<Double> overrideGimbalYValues) {
        double currentThrust = state.getDouble("thrust");
        HashSet<Action> possibleActions = new HashSet<>();

        if (simulationType == SimulationType._1D) {
            overrideGimbalXValues = new HashSet<>(Arrays.asList(0.0));
            overrideGimbalYValues = new HashSet<>(Arrays.asList(0.0));
        } else if (simulationType == SimulationType._2D) {
            // overrideGimbalYValues = new HashSet<>(Arrays.asList(0.0, Math.PI + 0.00001f));
        }

        HashMap<String, float[]> actionDefinition = state.definition.get("actionDefinition");

        Set<String> actionFields = (actionDefinition).keySet();

        HashSet<Double> possibleThrustValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideThrustValues.size() > 0) possibleThrustValues = overrideThrustValues;
        else {
            if (actionFields.contains("thrust"))
                possibleThrustValues = generatePossibleActionValues(currentThrust, actionDefinition.get("thrust"));
        }

        String symmetryAxis = "";

        HashSet<Double> possibleGimbalXValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideGimbalXValues.size() > 0) possibleGimbalXValues = overrideGimbalXValues;
        else {
            if (actionFields.contains("gimbalX"))
                possibleGimbalXValues = generatePossibleActionValues((float)state.getDouble("gimbalX"), actionDefinition.get("gimbalX"));
            if (actionFields.contains("gimbal")) {
                symmetryAxis = "X";
                possibleGimbalXValues = generatePossibleActionValues((float) state.getDouble("gimbalX"), actionDefinition.get("gimbal"));
            }
        }

        HashSet<Double> possibleGimbalYValues = new HashSet<>(Collections.singletonList(0.0));
        if (overrideGimbalYValues.size() > 0) possibleGimbalYValues = overrideGimbalYValues;
        else {
            if (actionFields.contains("gimbalY"))
                possibleGimbalYValues = generatePossibleActionValues((float)state.getDouble("gimbalY"), actionDefinition.get("gimbalY"));
            if (actionFields.contains("gimbal")) {
                symmetryAxis = "Y";
                possibleGimbalYValues = generatePossibleActionValues((float) state.getDouble("gimbalY"), actionDefinition.get("gimbal"));
            }
        }

        for (double possibleThrust: possibleThrustValues) {
            for (double possibleGimbalX: possibleGimbalXValues) {
                for (double possibleGimbalY: possibleGimbalYValues) {
                    Action action = new Action(possibleThrust, possibleGimbalX, possibleGimbalY, state.definition);
                    if (action.definition.get("meta").containsKey("symmetry")) {
                        String axes = (String)action.definition.get("meta").get("symmetry");
                        String[] symmetryAxes = axes.split(",");
                        for (String axis: symmetryAxes) {
                            action.setSymmetry(axis, symmetryAxis);
                        }
                    }
                    possibleActions.add(action);
                }
            }
        }

        return possibleActions;
    }

    // policy management

    private CoupledActions run_policy(CoupledStates coupledStates, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        double currentThrust = coupledStates.get(0).getDouble("thrust");
        CoupledActions coupledActions = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            State lastState = getLastStateOrNull(SAPrimary);
            State state = coupledStates.get(0);
            if (state.equals(lastState)) { return null; }
            Action action = policy(state, generatePossibleActions(state), primaryMethod);
            coupledActions = new CoupledActions(action, action, action);
            addStateActionTupleIfNotDuplicate(new StateActionTuple(state, action), SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {

            State lastLanderState = getLastStateOrNull(SAPrimary);
            Action bestLanderAction = getLastActionOrNull(SAPrimary);
            State landerState = coupledStates.get(0);
            if (!OptimizedMap.equivalentState(landerState, lastLanderState)) {
                bestLanderAction = policy(landerState, generatePossibleActions(landerState), primaryMethod);
                addStateActionTupleIfNotDuplicate(new StateActionTuple(landerState, bestLanderAction), SAPrimary);
            }

            // propagate thrust selection
            double thrust = bestLanderAction.getDouble("thrust");
            coupledStates.get(0).setDouble("thrust", thrust);
            coupledStates.get(1).setDouble("thrust", thrust);
            coupledStates.get(2).setDouble("thrust", thrust);

            // gimbal action selection below

            State lastGimbalXState = getLastStateOrNull(SAGimbalX);
            Action bestGimbalXAction = getLastActionOrNull(SAGimbalX);
            // calculate gimbalX
            State gimbalXState = coupledStates.get(1);
            if (!OptimizedMap.equivalentState(gimbalXState, lastGimbalXState)) {
                HashSet<Action> possibleActions = generatePossibleActions(gimbalXState, new HashSet<>(Arrays.asList(thrust)), new HashSet<>(), new HashSet<>(Arrays.asList(0.0)));
                if (Math.abs(coupledStates.get(0).getDouble("positionX")) >= 4.0) {
                    bestGimbalXAction = policy(gimbalXState, possibleActions, secondaryMethod);
                } else {
                    bestGimbalXAction = policy(gimbalXState, possibleActions, tertiaryMethod);
                }
                bestGimbalXAction.setDouble("gimbal", bestGimbalXAction.getDouble("gimbalX"));
                addStateActionTupleIfNotDuplicate(new StateActionTuple(gimbalXState, bestGimbalXAction), SAGimbalX);
            }

            State lastGimbalYState = getLastStateOrNull(SAGimbalY);
            Action bestGimbalYAction = getLastActionOrNull(SAGimbalY);
            // calculate gimbalY
            State gimbalYState = coupledStates.get(2);
            if (!OptimizedMap.equivalentState(gimbalYState, lastGimbalYState)) {
                HashSet<Action> possibleActions = generatePossibleActions(gimbalYState, new HashSet<>(Arrays.asList(thrust)), new HashSet<>(Arrays.asList(0.0)), new HashSet<>());
                if (Math.abs(coupledStates.get(0).getDouble("positionY")) >= 4.0) {
                    bestGimbalYAction = policy(gimbalYState, possibleActions, secondaryMethod);
                } else {
                    bestGimbalYAction = policy(gimbalYState, possibleActions, tertiaryMethod);
                }
                bestGimbalYAction.setDouble("gimbal", bestGimbalYAction.getDouble("gimbalY"));
                addStateActionTupleIfNotDuplicate(new StateActionTuple(gimbalYState, bestGimbalYAction), SAGimbalY);
            }

            if ((bestGimbalXAction != null) && (Math.abs(bestGimbalXAction.getDouble("gimbal")) > 200.0)) {
                System.out.println("Something is fucked up with this gimbal (X): " + bestGimbalXAction.getDouble("gimbal"));
            }
            if ((bestGimbalYAction != null) && (Math.abs(bestGimbalYAction.getDouble("gimbal")) > 200.0)) {
                System.out.println("Something is fucked up with this gimbal (Y): " + bestGimbalYAction.getDouble("gimbal"));
            }
            coupledActions = OptimizedMap.combineCoupledTripleActions(bestLanderAction, bestGimbalXAction, bestGimbalYAction);
        }
        return coupledActions;
    }

    private void addStateActionTupleIfNotDuplicate(StateActionTuple stateActionTuple, ArrayList<StateActionTuple> SA) {
        if (SA.size() == 0)
            SA.add(stateActionTuple);
        else if (!OptimizedMap.equivalentState(SA.get(SA.size() - 1).state, stateActionTuple.state)) {
            SA.add(stateActionTuple);
        }
    }

    public CoupledStates generateNewCoupledStates(SimulationStatus status) {
        State primaryState = new State(status, primaryMethod.definition);
        State secondaryState = null;
        State tertiaryState = null;
        if (Math.abs(primaryState.getDouble("positionX")) >= 4.0)
            secondaryState =  new State(status, secondaryMethod.definition);
        else
            secondaryState =  new State(status, tertiaryMethod.definition);
        if (Math.abs(primaryState.getDouble("positionY")) >= 4.0)
            tertiaryState =  new State(status, secondaryMethod.definition);
        else
            tertiaryState =  new State(status, tertiaryMethod.definition);

        return new CoupledStates(primaryState, secondaryState, tertiaryState);
    }

    public void updateStateBeforeNextAction(CoupledStates coupledStates, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (!SAPrimary.isEmpty()) {
                Action lastAction = SAPrimary.get(SAPrimary.size() - 1).action;
                State state = coupledStates.get(0);
                state.set("thrust", lastAction.get("thrust"));
                state.set("gimbalX", lastAction.get("gimbalX"));
                state.set("gimbalY", lastAction.get("gimbalY"));
            }
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (!SAPrimary.isEmpty()) {
                Action lastLanderAction = SAPrimary.get(SAPrimary.size() - 1).action;
                State primaryState = coupledStates.get(0);
                lastLanderAction.applyDefinitionValuesToState(primaryState);
                /*
                // DYNAMIC
                for (Object objectField: lastLanderAction.definition.get("actionDefinition").keySet()) {
                    String stateField = (String)objectField;
                    state.set(stateField, lastLanderAction.get(stateField));
                }*/
            }
            if (!SAGimbalX.isEmpty()) {
                Action lastGimbalXAction = SAGimbalX.get(SAGimbalX.size() - 1).action;
                State gimbalXState = coupledStates.get(1);
                lastGimbalXAction.applyDefinitionValuesToState(gimbalXState);
            }
            if (!SAGimbalY.isEmpty()) {
                Action lastGimbalYAction = SAGimbalY.get(SAGimbalY.size() - 1).action;
                State gimbalYState = coupledStates.get(2);
                lastGimbalYAction.applyDefinitionValuesToState(gimbalYState);
            }
        }
    }

    public CoupledActions generateAction(CoupledStates state, ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY) {
        return run_policy(state, SAPrimary, SAGimbalX, SAGimbalY);
    }

    public CoupledActions generateAction(CoupledStates state, ArrayList<StateActionTuple> SA) {
        return generateAction(state, SA, new ArrayList<>(), new ArrayList<>());
    }

    private Action policy(State state, HashSet<Action> possibleActions, ModelBaseImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();
        //bestActions.add(new Action(state.getDouble("thrust"), state.getDouble("gimbalX"), state.getDouble("gimbalY")));

        float explorationPercentage = method.getExplorationPercentage();

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            //try {
                float v = method.valueFunction(state, action);
                if (greedy) {
                    if (v > val) {
                        // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                        val = v;
                        bestActions = new HashSet<>();
                        bestActions.add(action);
                    } else if (v == val) {
                        // value is equal to other best value.  Add to bestAction ArrayList.
                        bestActions.add(action);
                    }
                } else {
                    bestActions.add(action);
                }
            //} catch (Exception e) {
            //    System.out.println("Failed to compute the value in the hashmap");
            //}
        }
        if (bestActions.size() == 0) {
            System.out.println("Size of best actions is 0.  This will cause failure - WHY IS IT HAPPENING?");
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY, int[] lastUpdateSizes) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (SAPrimary.size() != lastUpdateSizes[0]) primaryMethod.updateStepFunction(SAPrimary);

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (stateHasChanged(SAPrimary))
                primaryMethod.updateStepLanderFunction(SAPrimary);

            if (simulationType != SimulationType._1D) {
                if (SAGimbalX.size() != lastUpdateSizes[1]) {
                    Action lastActionX = getLastActionOrNull(SAGimbalX);
                    if ((lastActionX != null) && (stateHasChanged(SAGimbalX))) {
                        String MDPName = (String) lastActionX.definition.get("meta").get("name");
                        if (MDPName.equals("reacher"))
                            secondaryMethod.updateStepReachingFunction(SAGimbalX);
                        else if (MDPName.equals("stabilizer"))
                            tertiaryMethod.updateStepStabilizerFunction(SAGimbalX);
                        else
                            System.out.println("Something is fucked up in the value function (X)");
                    }
                }

                if (SAGimbalY.size() != lastUpdateSizes[2]) {
                    Action lastActionY = getLastActionOrNull(SAGimbalY);
                    if ((lastActionY != null) && (stateHasChanged(SAGimbalY))) {
                        String MDPName = (String) lastActionY.definition.get("meta").get("name");
                        if (MDPName.equals("reacher"))
                            secondaryMethod.updateStepReachingFunction(SAGimbalY);
                        else if (MDPName.equals("stabilizer"))
                            tertiaryMethod.updateStepStabilizerFunction(SAGimbalY);
                        else
                            System.out.println("Something is fucked up in the value function (Y)");
                    }
                }
            }
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> SAPrimary, ArrayList<StateActionTuple> SAGimbalX, ArrayList<StateActionTuple> SAGimbalY, TerminationBooleanTuple terminationBooleanTuple) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateTerminalFunction(SAPrimary);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if ((simulationType == SimulationType._1D) || (terminationBooleanTuple.landerSucceeded())) {
                System.out.println("Number of unique lander states: " + SAPrimary.size());
                primaryMethod.updateTerminalLanderFunction(SAPrimary);
            }
            if (simulationType != SimulationType._1D) {

                ArrayList<StateActionTuple> SAGimbalXReacher = new ArrayList<>();
                ArrayList<StateActionTuple> SAGimbalXStabilizer = new ArrayList<>();
                for (StateActionTuple stateActionTuple: SAGimbalX) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalXReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalXStabilizer.add(stateActionTuple);
                }

                ArrayList<StateActionTuple> SAGimbalYReacher = new ArrayList<>();
                ArrayList<StateActionTuple> SAGimbalYStabilizer = new ArrayList<>();
                for (StateActionTuple stateActionTuple: SAGimbalX) {
                    String MDPName = (String)stateActionTuple.action.definition.get("meta").get("name");
                    if (MDPName.equals("reacher"))
                        SAGimbalYReacher.add(stateActionTuple);
                    else if (MDPName.equals("stabilizer"))
                        SAGimbalYStabilizer.add(stateActionTuple);
                }

                double updateOrder = randomGenerator.nextDouble();
                if (updateOrder < 0.5) {
                    terminalUpdateLogic(secondaryMethod, "X", SAGimbalXReacher);
                    terminalUpdateLogic(tertiaryMethod, "X", SAGimbalXStabilizer);
                    terminalUpdateLogic(secondaryMethod, "Y", SAGimbalYReacher);
                    terminalUpdateLogic(tertiaryMethod, "Y", SAGimbalYStabilizer);
                } else {
                    terminalUpdateLogic(secondaryMethod, "Y", SAGimbalYReacher);
                    terminalUpdateLogic(tertiaryMethod, "Y", SAGimbalYStabilizer);
                    terminalUpdateLogic(secondaryMethod, "X", SAGimbalXReacher);
                    terminalUpdateLogic(tertiaryMethod, "X", SAGimbalXStabilizer);
                }
            }
        }
    }

    private void terminalUpdateLogic(ModelBaseImplementation method, String axis, ArrayList<StateActionTuple> SA) {
        String methodName = (String)method.definition.get("meta").get("name");
        // System.out.println("Number of unique " + methodName + " states (" + axis + "): " + SA.size());
        method.updateTerminalReachingFunction(SA);
    }

    public void resetValueFunctionTable() {
        setValueFunctionTable(new OptimizedMap());
    }

    public OptimizedMap getValueFunctionTable() {
        return primaryMethod.getValueFunctionTable();
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        primaryMethod.setValueFunctionTable(valueFunctionTable);
        secondaryMethod.setValueFunctionTable(valueFunctionTable);
        tertiaryMethod.setValueFunctionTable(valueFunctionTable);
    }

    public ModelBaseImplementation getPrimaryMethod() {
        return primaryMethod;
    }

    public void setPrimaryMethod(ModelBaseImplementation method) {
        // conserve the instance of the OptimizedMap
        OptimizedMap valueFunctionTable = primaryMethod.getValueFunctionTable();
        primaryMethod = method;
        secondaryMethod = primaryMethod;
        tertiaryMethod = primaryMethod;
        method.setValueFunctionTable(valueFunctionTable);
    }

    public CoupledActions getLastAction(ArrayList<StateActionTuple> episodeStateActionsPrimary, ArrayList<StateActionTuple> episodeStateActionsGimbalX, ArrayList<StateActionTuple> episodeStateActionsGimbalY) {
        if (episodeStateActionsPrimary.size() == 0)
            return null;

        CoupledActions coupledActions = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            Action action = episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action;
            coupledActions = new CoupledActions(action, action, action);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            coupledActions = OptimizedMap.combineCoupledTripleActions(
                    episodeStateActionsPrimary.get(episodeStateActionsPrimary.size() - 1).action,
                    episodeStateActionsGimbalX.get(episodeStateActionsGimbalX.size() - 1).action,
                    episodeStateActionsGimbalY.get(episodeStateActionsGimbalY.size() - 1).action
            );
        }
        return coupledActions;
    }


    private boolean stateHasChanged(ArrayList<StateActionTuple> episodeStateActions){
        State lastStateOrNull = getLastStateOrNull(episodeStateActions);
        if (lastStateOrNull == null) return false;
        State lastLastStateOrNull = getLastLastStateOrNull(episodeStateActions);
        return !lastStateOrNull.equals(lastLastStateOrNull);
    }

    private State getLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).state;
    }

    private State getLastLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.size() <= 1) ? null : episodeStateActions.get(episodeStateActions.size() - 2).state;
    }

    private Action getLastActionOrNull(ArrayList<StateActionTuple> episodeStateActions){
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).action;
    }


    @FunctionalInterface
    interface TriFunction<A,B,C,R> {

        R apply(A a, B b, C c);

        default <V> TriFunction<A, B, C, V> andThen(
                Function<? super R, ? extends V> after) {
            Objects.requireNonNull(after);
            return (A a, B b, C c) -> after.apply(apply(a, b, c));
        }
    }
}




