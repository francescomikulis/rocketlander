package net.sf.openrocket.simulation.extension.impl;

import java.util.*;
import java.io.Serializable;

import com.sun.istack.internal.NotNull;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import static net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class StateActionTuple implements Serializable {
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
        if (state.definition != action.definition) {
            System.out.println("State and action in same tuple have different definitions");
            this.action.definition = this.state.definition;
        }
        // THIS IS USELESS!
        // tupleRunAllDefinitionSpecial(this.state.definition);
    }

    private void tupleRunAllDefinitionSpecial(MDPDefinition definition) {
        // formula logic

        if (definition.symmetryAxes != null) {
            String stateSym = state.symmetry;
            String actionSym = action.symmetry;
            if ((stateSym == null) && (actionSym == null)) {
                System.out.println("SYMMETRY MUST BE SET IN EITHER ACTION OR STATE!");
            }
            else if ((stateSym != null) && (actionSym != null) && (!stateSym.equals(actionSym))){
                System.out.println("DIFFERENT VALUES OF SYMMETRY ARE INCOMPATIBLE!");
            }
            else if (stateSym != null) action.symmetry = state.symmetry;
            else if (actionSym != null) state.symmetry = action.symmetry;
        }

        if (state.symmetry != null) {
            for (Map.Entry<String, String> entry: definition.symmetryFormulas.get(state.symmetry).entrySet()) {
                String assignToField = entry.getKey();
                String fromField = entry.getValue();
                this.state.setDouble(assignToField, evaluateBestGuessAssignment(fromField, state, action));
                this.action.setDouble(assignToField, evaluateBestGuessAssignment(fromField, action, state));
            }
        }

        if (definition._formulas != null) {
            for (Map.Entry<String, Formula> entry: definition._formulas.entrySet()) {
                String assignToField = entry.getKey();
                Formula formula = entry.getValue();
                this.state.setDouble(assignToField, evaluateFormulaBestGuess(formula, state, action));
                this.action.setDouble(assignToField, evaluateFormulaBestGuess(formula, action, state));
            }
        }
    }

    private double evaluateBestGuessAssignment(String field, StateActionClass primary, StateActionClass fallback) {
        if (primary.valueMap.containsKey(field)) return primary.getDouble(field);
        else{
            if (fallback.valueMap.containsKey(field)) return fallback.getDouble(field);
            else System.out.println("MAJOR ISSUE IN MISSING PROPERTY: " + field);
        }
        return -1;
    }

    @Override
    public String toString() {
        String stateString = (state != null) ? state.toString() : "";
        String actionString = (action != null) ? action.toString() : "";
        return ("(" + stateString + ", " + actionString + ")");
    }

    @Override
    public int hashCode() {
        return state.hashCode() + action.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        StateActionTuple other = (StateActionTuple) obj;
        return this.action.equals(other.action) && this.state.equals(other.state);
    }

    public StateActionTuple deepcopy() {
        return new StateActionTuple(state.deepcopy(), action.deepcopy());
    }

    public static class StateActionClass implements Serializable {
        public HashMap<String, Integer> valueMap = new HashMap<>();
        public MDPDefinition definition;
        public String symmetry = null;

        protected void runMDPDefinitionFormulas() {
            runMDPDefinitionFormulas(false);
        }
        protected void runMDPDefinitionFormulas(boolean skipFormulas) {
            if (symmetry != null) {
                for (Map.Entry<String, String> entry: definition.symmetryFormulas.get(symmetry).entrySet()) {
                    setDouble(entry.getKey(), getDouble(entry.getValue()));
                }
            }

            if (skipFormulas) return;
            if ((definition != null) && (definition._formulas != null)) {
                for (Map.Entry<String, Formula> entry: definition._formulas.entrySet()) {
                    setDouble(entry.getKey(), evaluateFormula(entry.getValue(), this));
                }
            }
        }

        public void setSymmetry(String axis) {
            setSymmetry(axis, false);
        }

        public void setSymmetry(String axis, boolean skipFormulas) {
            symmetry = axis;
            if (symmetry == null) return;
            runMDPDefinitionFormulas(skipFormulas);
        }

        public static int group_by_precision(double value, double precision) {
            return (int) Math.floor(value / precision + 0.5);  // shift of 0.5 is required to create symmetrical values
        }

        /**
         * e.g. symmetric values around 0 [-10, 10]
         *
         * Without symmetric grouping:
         *  (-10, -5) --> -2
         *  ( -5, -0) --> -1
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         *
         * With symmetric grouping [if (result < 0) result += 1;]
         *  (-10, -5) --> -2 + 1 = -1
         *  ( -5, -0) --> -1 + 1 = 0
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         */
        protected static int group_by_precision_symmetric(double value, double precision) {
            int result = (int) Math.floor((double)value / (double)precision);
            if (result < 0)
                result += 1;
            return result;
        }

        public int get(String field) {
            Object result = valueMap.get(field);
            if (result == null) return 0;
            return (int)result;
        }

        public static int get(StateActionClass object, String field) {
            return object.get(field);
        }

        public double getDouble(String field) {
            if (definition == null) {
                return get(field) * 0.0000001f;
            } else {
                return get(field) * definition.precisions.getOrDefault(field, 0.0000001f) + definition.rangeShifts.getOrDefault(field, 0.0f);
            }
        }

        public StateActionClass set(String field, int value) {
            valueMap.put(field, value);
            return this;
        }

        public static StateActionClass set(StateActionClass object, String field, int value){
            object.valueMap.put(field, value);
            return object;
        }

        public StateActionClass setDouble(String field, double value) {
            if (definition == null) {
                float precision = 0.0000001f;
                valueMap.put(field, group_by_precision(value, precision));
            } else {
                float precision = definition.precisions.getOrDefault(field, 0.0000001f);
                valueMap.put(field, group_by_precision(value - definition.rangeShifts.getOrDefault(field, 0.0f), precision));
            }
            return this;
        }

        protected String stringifyStateOrAction(){
            String simpleClassName = this.getClass().getSimpleName();
            StringBuilder stringBuilder = new StringBuilder();
            stringBuilder.append(simpleClassName);
            stringBuilder.append("(");

            String[] keys = null;
            if (simpleClassName.toLowerCase().equals("state"))
                keys = definition.stateDefinitionFields;
            else // if (simpleClassName.toLowerCase().equals("action"))
                keys = definition.actionDefinitionFields;
            for (String field: keys) {
                stringBuilder.append(field);
                stringBuilder.append(": ");
                stringBuilder.append(this.getDouble(field));
                stringBuilder.append(" -> ");
                stringBuilder.append(this.get(field));
                stringBuilder.append(", ");
            }
            stringBuilder.append(")");
            String resultString = stringBuilder.toString();
            return resultString.replace(", )", ")");
        }

        public static void applyDeepcopy(StateActionClass fromObject, StateActionClass toObject, MDPDefinition definition) {
            toObject.definition = definition;
            toObject.valueMap = new HashMap<>();
            if (fromObject.symmetry != null)
                toObject.symmetry = fromObject.symmetry + "";
            for (Map.Entry<String, Integer> entry: fromObject.valueMap.entrySet()) {
                toObject.valueMap.put(entry.getKey() + "", entry.getValue() + 0);
            }
            if (fromObject.definition == toObject.definition) {
                toObject.runMDPDefinitionFormulas();
            } else {
                System.out.println("NEVER DO THIS!!!");
            }
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public State(SimulationStatus status){
            if (status == null) return;
            constructorCode(status);
        }

        public State(SimulationStatus status, MDPDefinition definition) {
            this.definition = definition;
            if (status == null) return;
            constructorCode(status);
        }

        private void constructorCode(SimulationStatus status) {
            if (status == null) return;

            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);

            setDouble("positionX", status.getRocketPosition().x);
            setDouble("positionY", status.getRocketPosition().y);
            setDouble("positionZ", status.getRocketPosition().z);

            setDouble("velocityX", status.getRocketVelocity().x);
            setDouble("velocityY", status.getRocketVelocity().y);
            setDouble("velocityZ", status.getRocketVelocity().z);
            //setVelocity(Math.signum(status.getRocketVelocity().z) * status.getRocketVelocity().length());

            // new angle approach
            double angleX = Math.asin(rocketDirection.x);
            double angleY = Math.asin(rocketDirection.y);

            double _45_deg = Math.asin(Math.sqrt(2)/2.0) - 0.0000001;
            if (rocketDirection.z <= 0.0 || Math.abs(angleX) >= _45_deg) {
                angleX = _45_deg * Math.signum(rocketDirection.x);
            }
            if (rocketDirection.z <= 0.0 || Math.abs(angleY) >= _45_deg) {
                angleY = _45_deg * Math.signum(rocketDirection.y);
            }
            setDouble("angleX", angleX);
            setDouble("angleY", angleY);
            // logic obtained directly from the rocket zenith orientation in OpenRocket
            setDouble("angleZ", Math.atan2(rocketDirection.z, MathUtil.hypot(rocketDirection.x, rocketDirection.y)));

            // original angle approach
            //setDouble("angleX", Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            //setDouble("angleZ", Math.acos(rocketDirection.z));

            // NOTE THAT THIS IS INTENTIONALLY FLIPPED!!!
            setDouble("angleVelocityX", status.getRocketRotationVelocity().y);
            setDouble("angleVelocityY", -status.getRocketRotationVelocity().x);

            setDouble("time", status.getSimulationTime());

            runMDPDefinitionFormulas();
        }


        @Override
        public int hashCode() {
            return MDPDefinition.computeIndexState(this);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            State other = (State) obj;
            boolean equivalent = true;
            for (Object objectField : definition.stateDefinitionFields) {
                String stateField = (String)objectField;
                equivalent = equivalent && this.get(stateField) == other.get(stateField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            return stringifyStateOrAction();
        }

        public State deepcopy() {
            return deepcopy(this.definition);
        }

        public State deepcopy(MDPDefinition definition) {
            State newState = new State(null);
            applyDeepcopy(this, newState, definition);
            return newState;
        }
    }

    public static class Action extends StateActionClass {
        public Action() {}
        public Action(HashMap<String, Integer> values, MDPDefinition definition) {
            constructorCodeInts(values, definition);
        }

        public Action(float thrust, float gimbalX, float gimbalY, float lateralThrustX, float lateralThrustY, MDPDefinition definition) {
            HashMap<String, Float> values = new HashMap<String, Float>() {{
                put("thrust", thrust);
                put("gimbalX", gimbalX);
                put("gimbalY", gimbalY);
                put("lateralThrustX", lateralThrustX);
                put("lateralThrustY", lateralThrustY);
            }};
            constructorCodeDoubles(values, definition);
        }

        private void constructorCodeInts(HashMap<String, Integer> values, MDPDefinition definition) {
            this.definition = definition;
            for (Map.Entry<String, Integer> entry: values.entrySet())
                set(entry.getKey(), entry.getValue());
            runMDPDefinitionFormulas();
        }

        private void constructorCodeDoubles(HashMap<String, Float> values, MDPDefinition definition) {
            if (definition != null)
                System.out.println("This constructor is not recommended because it loses precision!!!");
            this.definition = definition;
            for (Map.Entry<String, Float> entry: values.entrySet())
                setDouble(entry.getKey(), entry.getValue());
            runMDPDefinitionFormulas();
        }

        @Override
        public int hashCode() {
            return MDPDefinition.computeIndexAction(this);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Action other = (Action) obj;
            boolean equivalent = true;
            for (Object objectField : definition.actionDefinitionFields) {
                String actionField = (String)objectField;
                equivalent = equivalent && this.get(actionField) == other.get(actionField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            return stringifyStateOrAction();
        }

        public Action deepcopy() {
            return deepcopy(this.definition);
        }

        public Action deepcopy(MDPDefinition definition) {
            Action newAction = new Action(0, 0, 0, 0, 0, this.definition);
            applyDeepcopy(this, newAction, definition);
            return newAction;
        }

        public void applyDefinitionValuesToState(State state) {
            HashSet<String> symmetryAxes = definition.symmetryAxesHashSet;

            for (String actionField: definition.actionDefinitionFields) {
                double actionFieldValue = getDouble(actionField);
                if ((symmetryAxes != null) && symmetryAxes.contains(actionField)) {  // double setting the value for certainty
                    state.setDouble(actionField + symmetry, actionFieldValue);
                }

                if (state.definition == definition) {
                    state.setDouble(actionField, actionFieldValue);
                } else { // different definitions! DO NOT OVERRIDE THE VALUE unless not part of the actionDefinition
                    if (!state.definition.actionDefinition.containsKey(actionField))
                        state.setDouble(actionField, actionFieldValue);
                }
            }
        }
    }

    public static class CoupledStates extends ArrayList<State> {
        boolean frozen = false;
        String toStringNames = null;
        public CoupledStates(State... _states) {
            for (State s : _states) {
                super.add(s);
            }
        }
        public CoupledStates deepcopy() {
            ArrayList<State> copiedStateArrayList = new ArrayList<>();
            for (State s: this) {
                copiedStateArrayList.add(s.deepcopy());
            }
            return new CoupledStates(copiedStateArrayList.toArray(new State[copiedStateArrayList.size()]));
        }
        public void freeze() { frozen = true; }
        public String toStringNames() {
            if (!frozen || toStringNames == null) {
                StringBuilder tempNames = new StringBuilder();
                for (State s: this)
                    tempNames.append(s.definition.name);
                toStringNames = tempNames.toString();
            }
            return toStringNames;
        }
        @Override
        public String toString() {
            return toStringNames();
        }
        @Override
        public boolean add(State state) {
            if (frozen) {
                System.out.println("Cannot add state to a frozen CoupledStates!");
                return false;
            }
            return super.add(state);
        }
        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            CoupledStates other = (CoupledStates) obj;
            if (this.size() != other.size()) return false;

            boolean equals = true;
            for (int i = 0; i < this.size(); i++)
                equals = equals && this.get(i).equals(other.get(i));
            return equals;
        }
    }

    public static class CoupledActions extends ArrayList<Action> {
        public Action thrustAction = null;
        public Action gimbalXAction = null;
        public Action gimbalYAction = null;
        public Action lateralThrustXAction = null;
        public Action lateralThrustYAction = null;

        public CoupledActions(Action... _actions) {
            constructorCode(_actions);
        }

        private void constructorCode(Action... _actions) {
            for (Action a : _actions) {
                add(a);
            }
        }

        @Override
        public boolean add(Action action) {
            if (action.definition == null) {
                thrustAction = action;
                gimbalXAction = action;
                gimbalYAction = action;
                lateralThrustXAction = action;
                lateralThrustYAction = action;
            } else {
                if (action.definition.actionDefinition.containsKey("thrust")) {
                    thrustAction = action;
                }
                if (action.definition.actionDefinition.containsKey("gimbalX") || ((action.symmetry != null) && action.symmetry.equals("X") && action.definition.actionDefinition.containsKey("gimbal"))) {
                    gimbalXAction = action;
                }
                if (action.definition.actionDefinition.containsKey("gimbalY") || ((action.symmetry != null) && action.symmetry.equals("Y") && action.definition.actionDefinition.containsKey("gimbal"))) {
                    gimbalYAction = action;
                }
                if (action.definition.actionDefinition.containsKey("lateralThrustX") || ((action.symmetry != null) && action.symmetry.equals("X") && action.definition.actionDefinition.containsKey("lateralThrust"))) {
                    lateralThrustXAction = action;
                }
                if (action.definition.actionDefinition.containsKey("lateralThrustY") || ((action.symmetry != null) && action.symmetry.equals("Y") && action.definition.actionDefinition.containsKey("lateralThrust"))) {
                    lateralThrustYAction = action;
                }
            }
            return super.add(action);
        }

        private Action getAction(String field) {
            switch (field) {
                case "thrust":
                    return this.thrustAction;
                case "gimbalX":
                    return this.gimbalXAction;
                case "gimbalY":
                    return this.gimbalYAction;
                case "lateralThrustX":
                    return this.lateralThrustXAction;
                case "lateralThrustY":
                    return this.lateralThrustYAction;
            }
            System.out.println("attempting to pull field not contained in actions: " + field);
            return this.thrustAction;
        }

        public double getDouble(String field) {
            Action action = getAction(field);
            if (action == null) return 0.0;
            return action.getDouble(field);
        }

        public void setDouble(String field, double value) {
            Action action = getAction(field);
            if (action == null) return;
            action.setDouble(field, value);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            CoupledActions other = (CoupledActions) obj;
            if (this.size() != other.size()) return false;

            boolean equals = true;
            for (int i = 0; i < this.size(); i++)
                equals = equals && this.get(i).equals(other.get(i));
            return equals;
        }
    }


    public static Quaternion getConjugateQuaternion(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    public static Coordinate convertRocketStatusQuaternionToDirection(SimulationStatus status) {
        return status.getRocketOrientationQuaternion().rotateZ();
    }
}

