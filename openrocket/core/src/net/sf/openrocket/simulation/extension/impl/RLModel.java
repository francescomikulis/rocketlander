package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;

import static net.sf.openrocket.simulation.extension.impl.OptimizedMap.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private static volatile RLModel instance;
    private Random randomGenerator = new Random();
    OptimizedMap valueFunctionTable;

    private LinkedHashMap<String, ModelBaseImplementation> methods;

    public String symmetryAxis2D = "X";
    public String symmetryAxis3D = "Y";
    public SimulationType simulationType = SimulationType._3D;

    enum SimulationType {
        _1D, _2D, _3D
    }

    private RLModel(){
        constructor(false);
    }

    private void constructor(boolean reset) {
        // this is the default constructor - these hashmap definition should be read from elsewhere

        constructor("X", "Y", SimulationType._3D, reset, getLanderDefinition(), getReacherDefinition(), getStabilizerDefinition());
        // constructor("X", "Y", SimulationType._3D, generalDefinition);
    }

    private void constructor(String symmetryAxis2D, String symmetryAxis3D, SimulationType simulationType, boolean reset, MDPDefinition ... definitions) {
        valueFunctionTable = null;
        this.symmetryAxis2D = symmetryAxis2D;
        this.symmetryAxis3D = symmetryAxis3D;
        this.simulationType = simulationType;
        this.methods = new LinkedHashMap<>();
        for (MDPDefinition definition: definitions) {
            methods.put(definition.name, definition.model);
        }
        // setValueFunctionTable
        if (reset) {
            this.valueFunctionTable = new OptimizedMap(methods);
        } else {
            this.valueFunctionTable = RLObjectFileStore.getInstance().readActionValueFunctionFromMethods(methods);
        }
    }

    public void resetValueFunctionTable() {
        valueFunctionTable.resetValueFunctionTable(methods);
        constructor(true);
    }

    public OptimizedMap getValueFunctionTable() {
        return valueFunctionTable;
    }

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

    public LinkedHashMap<String, ModelBaseImplementation> getMethods() {
        return methods;
    }

    public Set<String> getMethodNames() {
        return methods.keySet();
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

        if ((definition._MDPSelectionFormulas != null) && definition._MDPSelectionFormulas.containsKey(MDPActionSelectionField)) {
            ArrayList<Object[]> advancedIfElseFormula = (definition._MDPSelectionFormulas.get(MDPActionSelectionField));

            String selectedMDPName = null;
            for (int i = 0; i < advancedIfElseFormula.size(); i += 2) {
                Formula formula = (Formula) advancedIfElseFormula.get(i)[0];
                if (formula.evaluate(state) != 0) {  // 0 is false, 1 is true
                    selectedMDPName = (String) advancedIfElseFormula.get(i)[1];
                    break;
                }
            }
            if (selectedMDPName == null) {
                if (advancedIfElseFormula.size() == 0) {
                    System.out.println("Unable to generate MDP selection!");
                } else {
                    // default to last one!
                    selectedMDPName = (String)advancedIfElseFormula.get(advancedIfElseFormula.size() - 1)[1];
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
                    System.out.println("Formula evaluation for child MDP selection did not evaluate correctly.  This should not be possible!");
                }
            } else {
                possibleActionInts.add(new ArrayList<>(generatePossibleActionValuesInts(state.get(realField), state.definition.actionDefinitionIntegers[index])));
            }
            index++;
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
                    State childState = new State(status, methods.get(selectedMDPName).definition);
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
        for (String key: methods.keySet()) { firstDefinition = methods.get(key).definition; break; }
        coupledStates.add(new State(status, firstDefinition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            State state = coupledStates.get(methodCounter);
            String methodName = state.definition.name;
            ModelBaseImplementation method = methods.get(methodName);

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
            for (State s: coupledStates)
                bestAction.applyDefinitionValuesToState(s);
            actions.add(bestAction);
            methodCounter += 1;
        }

        coupledStates.freeze();

        // need to update the formulas after getting the new data
        for (State s: coupledStates)
            s.runMDPDefinitionFormulas();

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

    private Action policy(State state, HashSet<Action> possibleActions, ModelBaseImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();

        float explorationPercentage = method.getExploration();

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
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
        }
        if (bestActions.size() == 0) {
            System.out.println("Size of best actions is 0.  This will cause failure - WHY IS IT HAPPENING?");
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, LinkedHashMap<String, Integer> lastUpdateSizes) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            ModelBaseImplementation method = entry.getValue();
            Formula reward = method.definition._reward;

            if (methods.get(methodName).definition.symmetryAxes == null) {
                methods.get(methodName).updateStepCustomFunction(SA.get(methodName), reward);
            } else {
                if (0.5 < randomGenerator.nextDouble()) {
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "X"), reward);
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                } else {
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "X"), reward);
                }
            }
        }
    }

    public void updateTerminalStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, TerminationBooleans terminationBooleans) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            ModelBaseImplementation method = entry.getValue();
            if (method.definition._terminalReward == null) continue;  // terminalReward not specified, skip method!
            Formula terminalReward = method.definition._terminalReward;
            Formula reward = method.definition._reward;

            if (methods.get(methodName).definition.symmetryAxes == null) {
                methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName), terminalReward, reward);
            } else {
                if (0.5 < randomGenerator.nextDouble()) {
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                } else {
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                }
            }
        }

        if (terminationBooleans.totalSuccess()) {
            System.out.print("+");
        } else {
            System.out.print("-");
        }
    }
}




