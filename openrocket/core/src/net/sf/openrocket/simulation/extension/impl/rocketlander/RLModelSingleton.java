package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.rocketlander.methods.*;

import java.util.*;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions.Expression;


public class RLModelSingleton {
    private static volatile RLModelSingleton instance;
    private Random randomGenerator = new Random();
    ValueFunctionManager valueFunctionManager;

    private LinkedHashMap<String, MDPDefinition> methods = new LinkedHashMap<>();

    public String symmetryAxis2D = "X";
    public String symmetryAxis3D = "Y";
    public SimulationType simulationType = SimulationType._3D;
    public SimulationInitVariation initVariation = SimulationInitVariation.all;

    public RLDataStoreSmartPlotValues dataStoreState;

    private StringBuilder stringBuilder = new StringBuilder();
    private boolean smartPrintBuffer = false;

    public enum SimulationType {
        _1D, _2D, _3D
    }

    public enum SimulationInitVariation {
        fixed, posVel, loc, posVelLoc, posVelAngle, all
    }

    private RLModelSingleton(){
        constructor();
    }

    private void constructor(MDPDefinition ... definitions) {
        // this is the default constructor - these hashmap definition should be read from elsewhere

        constructor(symmetryAxis2D, symmetryAxis3D, simulationType, definitions);
        // constructor("X", "Y", SimulationType._3D, generalDefinition);
    }

    public void setDefinitions(ArrayList<MDPDefinition> definitions) {
        dataStoreState = new RLDataStoreSmartPlotValues();
        boolean actualChange = false;
        HashSet<String> newDefinitionNames = new HashSet<>();
        for (MDPDefinition definition: definitions) {
            newDefinitionNames.add(definition.name);
            if (!methods.containsKey(definition.name)) {
                actualChange = true;
                definition.setValueFunction(null);
                continue;
            }
            for (BaseMethodImplementation method: methods.get(definition.name).models) {
                double originalExploration = method.definition.exploration;
                method.definition.exploration = definition.exploration;
                String newDefinitionString = MDPDefinition.toJsonString(method.definition).replaceAll(" ", "");
                String oldDefinitionString = MDPDefinition.toJsonString(definition).replaceAll(" ", "");
                method.definition.exploration = originalExploration;

                // only re-use if the only modified field was at most the exploration parameter
                if (newDefinitionString.equals(oldDefinitionString)) {
                    definition.setValueFunction(method.definition.valueFunction);
                } else {
                    actualChange = true;
                    definition.setValueFunction(null);
                }
            }
        }
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            if (!newDefinitionNames.contains(entry.getKey())) {
                actualChange = true;
                break;
            }
        }
        if (actualChange) {
            methods = new LinkedHashMap<>();
            constructor(symmetryAxis2D, symmetryAxis3D, simulationType, definitions.toArray(new MDPDefinition[definitions.size()]));
        }
    }

    private void constructor(String symmetryAxis2D, String symmetryAxis3D, SimulationType simulationType, MDPDefinition ... definitions) {
        dataStoreState = new RLDataStoreSmartPlotValues();
        valueFunctionManager = null;
        this.symmetryAxis2D = symmetryAxis2D;
        this.symmetryAxis3D = symmetryAxis3D;
        this.simulationType = simulationType;

        // merge previously stored definitions because still valid!
        MDPDefinition[] mergedDefinitions = new MDPDefinition[methods.size() + definitions.length];
        int mergedIndex = 0;
        for (MDPDefinition definition: methods.values()) {
            mergedDefinitions[mergedIndex] = definition;
            mergedIndex++;
        }
        for (MDPDefinition definition: definitions) {
            mergedDefinitions[mergedIndex] = definition;
            mergedIndex++;
        }

        for (MDPDefinition definition: sortMDPDefinitionsByPriority(mergedDefinitions)) {
            methods.put(definition.name, definition);
        }
        // setValueFunctionManager
        this.valueFunctionManager = new ValueFunctionManager(methods);
    }

    private MDPDefinition[] sortMDPDefinitionsByPriority(MDPDefinition[] definitions) {
        MDPDefinition[] sortedMDPDefinitions = new MDPDefinition[definitions.length];
        int minPriority = Integer.MAX_VALUE;
        int maxPriority = Integer.MIN_VALUE;
        for (MDPDefinition definition: definitions) {
            minPriority = Math.min(minPriority, definition.priority);
            maxPriority = Math.max(maxPriority, definition.priority);
        }
        int currentIndex = 0;
        for (int i = minPriority; i <= maxPriority; i++) {
            for (MDPDefinition definition: definitions) {
                if (definition.priority == i) {
                    sortedMDPDefinitions[currentIndex] = definition;
                    currentIndex++;
                }
            }
        }
        return sortedMDPDefinitions;
    }

    public void resetValueFunctionManager(MDPDefinition[] definitions) {
        valueFunctionManager.resetValueFunctionManager(definitions);
        for (MDPDefinition definition: definitions) {
            methods.remove(definition.name);
        }
        constructor(definitions);
    }

    public ValueFunctionManager getValueFunctionManager() {
        return valueFunctionManager;
    }

    public static RLModelSingleton getInstance() {
        if (instance == null) { // first time lock
            synchronized (RLModelSingleton.class) {
                if (instance == null) {  // second time lock
                    instance = new RLModelSingleton();
                }
            }
        }
        return instance;
    }

    public LinkedHashMap<String, MDPDefinition> getMethods() {
        return methods;
    }

    public LinkedHashMap<String, ArrayList<StateActionTuple>> initializeEpisodeStateActions() {
        LinkedHashMap<String, ArrayList<StateActionTuple>> episodeStateActions = new LinkedHashMap<>();
        for (String method: methods.keySet()) {
            episodeStateActions.put(method, new ArrayList<>());
        }
        return episodeStateActions;
    }

    public void setupSimulationTypeBasedOnMDPDefinitions(SimulationStatus status) {
        SimulationType updatedSimType = simulationType;
        // select the smallest available sim type if forcefully defined in MDPDefinition name
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            MDPDefinition definition = entry.getValue();
            if (definition.name.contains("_1D")) {
                updatedSimType = SimulationType._1D;
            }
            else if (definition.name.contains("_2D") && (updatedSimType != SimulationType._1D)) {
                updatedSimType = SimulationType._2D;
            }
            else if (definition.name.contains("_3D") && (updatedSimType != SimulationType._1D) && (updatedSimType != SimulationType._2D)) {
                updatedSimType = SimulationType._3D;
            }
        }
        simulationType = updatedSimType;
    }

    private HashSet<Integer> generatePossibleActionValuesInts(int value, int[] definition) {
        HashSet<Integer> possibleActionValues = new HashSet<>();
        for (int i = definition[0]; i <= definition[1]; i++) {
            possibleActionValues.add(i);
        }
        return possibleActionValues;
    }

    private HashSet<Integer> generatePossibleActionValuesMDPInts(State state, String MDPActionSelectionField) {
        MDPDefinition definition = state.definition;
        HashSet<Integer> possibleActionValues = new HashSet<>();

        if ((definition._MDPSelectionExpressions != null) && definition._MDPSelectionExpressions.containsKey(MDPActionSelectionField)) {
            ArrayList<Object[]> advancedIfElseExpression = (definition._MDPSelectionExpressions.get(MDPActionSelectionField));

            String selectedMDPName = null;
            for (int i = 0; i < advancedIfElseExpression.size(); i += 2) {
                Expression expression = (Expression) advancedIfElseExpression.get(i)[0];
                String tempMDPName = (String) advancedIfElseExpression.get(i)[1];
                if (methods.containsKey(tempMDPName) && (expression.evaluate(state) != 0)) {  // 0 is false, 1 is true
                    selectedMDPName = tempMDPName;
                    break;
                }
            }
            if (selectedMDPName == null) {
                if (advancedIfElseExpression.size() == 0) {
                    System.out.println("Unable to generate MDP selection!");
                } else {
                    // System.out.println("Defaulting MDP SELECTION - BAD.");
                    // default to last one!
                    selectedMDPName = (String)advancedIfElseExpression.get(advancedIfElseExpression.size() - 1)[1];
                    // if the last one isn't defined, restart the switch from the top until one is defined
                    if (!methods.containsKey(selectedMDPName)) {
                        for (int i = 0; i < advancedIfElseExpression.size(); i += 2) {
                            selectedMDPName = (String) advancedIfElseExpression.get(i)[1];
                            if (methods.containsKey(selectedMDPName)) {
                                break;
                            }
                        }
                    }
                }
            }
            // overrides the traditional options calculations of that MDP Field Name
            if (selectedMDPName != null)
                possibleActionValues.add(definition.childrenMDPIntegerOptions.get(selectedMDPName));
        } else {
            String[] MDPNames = definition.childrenMDPOptions.get(MDPActionSelectionField);
            for (String MDPName : MDPNames) {
                // convert MDPName to fixed integer number
                possibleActionValues.add((Integer) definition.childrenMDPIntegerOptions.get(MDPName));
            }
        }
        return possibleActionValues;
    }

    private boolean incrementIndeces(int[] indeces, int[] maxIndeces) {
        return _incrementIndeces(indeces, maxIndeces, indeces.length - 1);
    }

    public boolean _incrementIndeces(int[] indeces, int[] maxIndeces, int position) {
        if (position == -1) return false;
        if (indeces[position] < maxIndeces[position]) {
            indeces[position] += 1;
            return true;
        }
        indeces[position] = 0;
        return _incrementIndeces(indeces, maxIndeces, position - 1);
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/

    public HashSet<Action> generatePossibleActions(State state) {
        ArrayList<ArrayList<Integer>> possibleActionInts = new ArrayList<>();

        HashSet<String> symmetryAxes = state.definition.symmetryAxesHashSet;
        String[] actionFields = state.definition.actionDefinitionFields;
        ArrayList<String> actionStrings = new ArrayList<>();  // these have actual components (e.g. gimbalX - NOT gimbal)
        int index = 0;
        for (String actionField: actionFields) {
            String realField = actionField;
            if (symmetryAxes != null) {
                if (symmetryAxes.contains(actionField))
                    realField += state.symmetry;
            }
            actionStrings.add(realField);
            if ((state.definition.childrenMDPOptions != null) && state.definition.childrenMDPOptions.containsKey(realField)) {
                ArrayList<Integer> childMDPOptions = new ArrayList<>(generatePossibleActionValuesMDPInts(state, realField));
                possibleActionInts.add(childMDPOptions);
                if (childMDPOptions.size() != 1) {
                    // System.out.println("Expression evaluation for child MDP selection did not evaluate correctly.  This should not be possible!");
                }
            } else {
                possibleActionInts.add(new ArrayList<>(generatePossibleActionValuesInts(state.get(realField), state.definition.actionDefinitionIntegers[index])));
            }
            index++;
        }

        if ((state.definition.childrenMDPOptions != null) && (state.definition.childrenMDPOptions.size() != 0)) {
            for (Map.Entry<String, String[]> entry: state.definition.childrenMDPOptions.entrySet()) {
                // if already choosing the child MDP in the actionDefinition skip this logic - else it's needed to still compute children
                if (state.definition.actionDefinition.containsKey(entry.getKey()))
                    continue;
                actionStrings.add(entry.getKey());
                ArrayList<Integer> childMDPOptions = new ArrayList<>(generatePossibleActionValuesMDPInts(state, entry.getKey()));
                possibleActionInts.add(childMDPOptions);
            }
        }

        int size = actionStrings.size();
        int[] indeces = new int[size];
        int[] maxIndeces = new int[size];
        for (int i = 0; i < size; i++) {
            indeces[i] = 0;
            maxIndeces[i] = possibleActionInts.get(i).size() - 1;
        }

        HashSet<Action> possibleActions = new HashSet<>();
        while (true) {
            HashMap<String, Integer> actionValueMap = new HashMap<>();
            for (int i = 0; i < size; i++) {
                actionValueMap.put(actionStrings.get(i), possibleActionInts.get(i).get(indeces[i]));
            }
            Action action = new Action(actionValueMap, state.definition);
            action.setSymmetry(state.symmetry, true);
            possibleActions.add(action);

            if (!incrementIndeces(indeces, maxIndeces)) {
                break;
            }
        }
        return possibleActions;
    }

    private void addHierarchicalMDPSelection(SimulationStatus status, CoupledStates coupledStates, Action action) {
        if (action.definition.childrenMDPOptions != null) {
            for (String MDPActionSelectionField : action.definition.childrenMDPOptions.keySet()) {
                int SelectedMDPID = action.get(MDPActionSelectionField);

                // reverse hashmap and find the MDP name
                String selectedMDPName = null;
                for (Map.Entry<String, Integer> internalEntry : action.definition.childrenMDPIntegerOptions.entrySet()) {
                    if (internalEntry.getValue() == SelectedMDPID) {
                        selectedMDPName = internalEntry.getKey();
                    }
                }

                // hack to 'pass down' the symmetry for mid-level MDPs
                if (action.definition.passDownSymmetryAxis) {
                    MDPActionSelectionField += action.symmetry;
                }

                boolean actuallyCreateNewState = true;
                String symmetryAxis = null;
                if (MDPActionSelectionField.contains("X")) symmetryAxis = "X";
                else if (MDPActionSelectionField.contains("Y")) symmetryAxis = "Y";
                if (symmetryAxis != null) {
                    if (simulationType == SimulationType._1D) {
                        actuallyCreateNewState = false;
                    }
                    if (simulationType == SimulationType._2D) {
                        if (!MDPActionSelectionField.contains(symmetryAxis2D)) {
                            actuallyCreateNewState = false;
                        }
                    }
                }
                if (selectedMDPName == null) actuallyCreateNewState = false;

                if (actuallyCreateNewState) {
                    State childState = new State(status, methods.get(selectedMDPName));
                    childState.setSymmetry(symmetryAxis);
                    coupledStates.add(childState);
                }
            }
        }
    }

    // policy management

    private Object[] run_policy(SimulationStatus status, LinkedHashMap<String, ArrayList<StateActionTuple>> SA) {
        ArrayList<Action> actions = new ArrayList<>();
        CoupledStates coupledStates = new CoupledStates();
        MDPDefinition firstDefinition = null;
        for (String key: methods.keySet()) { firstDefinition = methods.get(key); break; }
        if (firstDefinition == null) return new Object[]{null, null};
        coupledStates.add(new State(status, firstDefinition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            State state = coupledStates.get(methodCounter);
            String methodName = state.definition.name;
            BaseMethodImplementation method = methods.get(methodName).models[0];

            String storageName = methodName;
            if (state.symmetry != null) storageName += state.symmetry;
            if (!SA.containsKey(storageName)) SA.put(storageName, new ArrayList<>());
            ArrayList<StateActionTuple> stateActionTuples = SA.get(storageName);

            State lastState = null;
            Action bestAction = null;
            if ((stateActionTuples != null) && !stateActionTuples.isEmpty()) {
                lastState = stateActionTuples.get(stateActionTuples.size() - 1).state;
                bestAction = stateActionTuples.get(stateActionTuples.size() - 1).action;
            }

            if (MDPDefinition.needToChooseNewAction(state, lastState, bestAction)) {
                HashSet<Action> possibleActions = generatePossibleActions(state);
                bestAction = policy(state, possibleActions, method);
                bestAction.setSymmetry(state.symmetry);
                addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestAction), stateActionTuples);
            }

            // dynamically adds entries to coupledStates - bad convention but required!
            addHierarchicalMDPSelection(status, coupledStates, bestAction);

            // propagate selection
            actions.add(bestAction);
            for (Action a: actions) {
                for (State s : coupledStates) {
                    a.applyDefinitionValuesToState(s);
                }
            }

            methodCounter += 1;
        }

        coupledStates.freeze();

        // need to update the expressions after getting the new data
        for (State s: coupledStates)
            s.runMDPDefinitionExpressions();

        CoupledActions coupledActions = new CoupledActions(actions.toArray(new Action[actions.size()]));

        return new Object[]{coupledStates, coupledActions};
    }

    private void addStateActionTupleIfNotDuplicate(StateActionTuple stateActionTuple, ArrayList<StateActionTuple> SA) {
        if (SA.size() == 0)
            SA.add(stateActionTuple);
        else if (!MDPDefinition.equivalentState(SA.get(SA.size() - 1).state, stateActionTuple.state)) {
            SA.add(stateActionTuple);
        }
    }

    public CoupledStates generateCoupledStatesBasedOnLastActions(SimulationStatus status, CoupledActions coupledActions) {
        CoupledStates coupledStates = new CoupledStates();
        coupledStates.add(new State(status, coupledActions.get(0).definition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            addHierarchicalMDPSelection(status, coupledStates, coupledActions.get(methodCounter));
            methodCounter += 1;
        }

        return coupledStates;
    }

    public Object[] generateStateAndActionAndStoreHistory(SimulationStatus status, LinkedHashMap<String, ArrayList<StateActionTuple>> SA) {
        return run_policy(status, SA);
    }

    private Action policy(State state, HashSet<Action> possibleActions, BaseMethodImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();

        float explorationPercentage = method.getExploration();

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        float stateReward = method.definition._reward.evaluateFloat(state);
        for (Action action: possibleActions) {
            float v = method.valueFunction(state, action);
            if (v == 0.0f) {
                v = stateReward + method.definition._reward.evaluateFloat(action);
            }
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
        }
        if (bestActions.size() == 0) {
            System.out.println("Size of best actions is 0.  This will cause failure - WHY IS IT HAPPENING?");
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, LinkedHashMap<String, Integer> lastUpdateSizes) {
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            for (BaseMethodImplementation method: entry.getValue().models) {
                Expression reward = method.definition._reward;

                if (method.definition.symmetryAxes == null) {
                    method.updateStepCustomFunction(SA.get(methodName), reward);
                } else {
                    if (0.5 < randomGenerator.nextDouble()) {
                        method.updateStepCustomFunction(SA.get(methodName + "X"), reward);
                        method.updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                    } else {
                        method.updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                        method.updateStepCustomFunction(SA.get(methodName + "X"), reward);
                    }
                }
            }
        }
    }

    public void updateTerminalStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, TerminationBooleans terminationBooleans) {
        for (Map.Entry<String, MDPDefinition> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            for (BaseMethodImplementation method: entry.getValue().models) {
                if (method.definition._terminalReward == null) continue;  // terminalReward not specified, skip method!
                Expression terminalReward = method.definition._terminalReward;
                Expression reward = method.definition._reward;

                if ((method.definition.symmetryAxes == null) || (method.definition.symmetryAxes.length == 0)) {
                    method.updateTerminalCustomFunction(SA.get(methodName), terminalReward, reward);
                } else {
                    if (0.5 < randomGenerator.nextDouble()) {
                        method.updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                        method.updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                    } else {
                        method.updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                        method.updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                    }
                }
            }
        }

        if (terminationBooleans.totalSuccess()) {
            stringBuilder.append('+');
        } else {
            stringBuilder.append('-');
        }
        if (!smartPrintBuffer) {
            printAndClearStringBuffer(false);
        }
    }

    public void printStatusInformationOnSingleSimTermination(SimulationStatus status) {
        if (!smartPrintBuffer)
            RocketLanderListener.printStatusInformation(status);
    }


    /* Interface actions for the SimulationPanel in the UI */

    public void setSmartPrintBuffer(boolean smartPrintBuffer) {
        if (stringBuilder.length() != 0)
            printAndClearStringBuffer();
        this.smartPrintBuffer = smartPrintBuffer;
    }

    public void printAndClearStringBuffer() {
        printAndClearStringBuffer(true);
    }

    public void printAndClearStringBuffer(boolean goNewline) {
        String result = stringBuilder.toString();
        stringBuilder.setLength(0);
        String successInfo = "";
        if (result.length() > 1) {
            int num_successes = 0;
            for (int i = 0; i < result.length(); i++) {
                if (result.charAt(i) == '+') num_successes++;
            }
            successInfo = " --> %" + (((double) num_successes) * 100.0 / (double) (result.length()));
        }
        if (goNewline)
            successInfo += "\n";
        System.out.print(result + successInfo);
    }

    /* Interface actions for the RLPanel in the UI */

    public void stepNextSimulationType() {
        RLModelSingleton.SimulationType newSimulationType = null;
        if (simulationType == RLModelSingleton.SimulationType._1D) {
            newSimulationType = RLModelSingleton.SimulationType._2D;
        } else if (simulationType == RLModelSingleton.SimulationType._2D) {
            newSimulationType = RLModelSingleton.SimulationType._3D;
        } else if (simulationType == RLModelSingleton.SimulationType._3D) {
            newSimulationType = RLModelSingleton.SimulationType._1D;
        }
        simulationType = newSimulationType;
    }

    public void stepNextSimulation2DAxis() {
        if (symmetryAxis2D.equals("X")) {
            symmetryAxis2D = "Y";
        } else {
            symmetryAxis2D = "X";
        }
    }

    public void stepNextInitialVariation() {
        SimulationInitVariation newInitVariation = null;
        if (initVariation == RLModelSingleton.SimulationInitVariation.fixed) {
            newInitVariation = RLModelSingleton.SimulationInitVariation.posVel;
        } else if (initVariation == RLModelSingleton.SimulationInitVariation.posVel) {
            newInitVariation = SimulationInitVariation.loc;
        } else if (initVariation == RLModelSingleton.SimulationInitVariation.loc) {
                newInitVariation = SimulationInitVariation.posVelLoc;
        } else if (initVariation == RLModelSingleton.SimulationInitVariation.posVelLoc) {
            newInitVariation = SimulationInitVariation.posVelAngle;
        } else if (initVariation == RLModelSingleton.SimulationInitVariation.posVelAngle) {
            newInitVariation = RLModelSingleton.SimulationInitVariation.all;
        } else if (initVariation == RLModelSingleton.SimulationInitVariation.all) {
            newInitVariation = RLModelSingleton.SimulationInitVariation.fixed;
        }
        initVariation = newInitVariation;
    }
}




