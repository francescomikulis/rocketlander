package net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;

class Constant extends Term {
    double value = Double.MAX_VALUE;
    public Constant(String identifier) {
        super(identifier);
    }
    public Constant(String identifier, double value) {
        super(identifier);
        this.value = value;
    }

    @Override
    public String toString() {
        if (sign == -1) return "-" + identifier;
        else return identifier;
    }

    public double evaluate(StateActionTuple.StateActionClass object) {
        if (value != Double.MAX_VALUE) return value;
        if (sign == -1) return -object.getDouble(identifier);
        else return object.getDouble(identifier);
    }

    public double evaluateBestGuess(StateActionTuple.StateActionClass primary, StateActionTuple.StateActionClass fallback) {
        if (value != Double.MAX_VALUE) return value;

        double value = 0;
        if (primary.valueMap.containsKey(identifier)) value = primary.getDouble(identifier);
        else {
            if (fallback.valueMap.containsKey(identifier)) value = fallback.getDouble(identifier);
            else System.out.println("MAJOR ISSUE IN MISSING PROPERTY: " + identifier);
        }

        if (sign == -1) return -value;
        else return value;
    }

    /** Pre-defined constant values **/

    public static double parseConstant(String partial_input) {
        if (partial_input.equals("PI")) return Math.PI;
        else if (partial_input.equals("E")) return Math.exp(1);
        else if (partial_input.equals("TRUE")) return 1.0;
        else if (partial_input.equals("FALSE")) return 0.0;
        return Double.parseDouble(partial_input);
    }
}