package net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;

import java.util.ArrayList;

class Function extends Term {
    ArrayList<Term> inputs;
    public Function(String identifier, ArrayList<Term> inputs) {
        super(identifier);
        this.inputs = inputs;
    }

    @Override
    public boolean equals(Object obj) {
        if (!super.equals(obj)) return false;
        Function other = (Function) obj;
        return this.inputs.equals(other.inputs);
    }

    @Override
    public int hashCode() {
        return identifier.hashCode() + inputs.size();
    }

    @Override
    public String toString() {
        String string = identifier;
        if (sign == -1) string = "-" + identifier;

        if (inputs != null) {
            ArrayList<String> stringArrayList = new ArrayList<>();
            for (Term input : inputs) {
                stringArrayList.add(input.toString());
            }
            string = string + "(" + String.join(",", stringArrayList) + ")";
        }
        return string;
    }

    public double evaluate(StateActionTuple.StateActionClass object) {
        ArrayList<Double> doubleInputs = new ArrayList<>();
        for (Term input: inputs) {
            doubleInputs.add(input.evaluate(object));
        }
        double result = mathematicalEvaluation(identifier, doubleInputs);
        if (sign == -1) return -result;
        else return result;
    }
    public double evaluateBestGuess(StateActionTuple.StateActionClass primary, StateActionTuple.StateActionClass fallback) {
        ArrayList<Double> doubleInputs = new ArrayList<>();
        for (Term input: inputs) {
            doubleInputs.add(input.evaluateBestGuess(primary, fallback));
        }
        double result = mathematicalEvaluation(identifier, doubleInputs);
        if (sign == -1) return -result;
        else return result;
    }






    private static double mathematicalEvaluation(String method, ArrayList<Double> inputs) {
        if (method.equals("")) return inputs.get(0);  // assignment
        else if (method.equals("Add")) return inputs.get(0) + inputs.get(1);
        else if (method.equals("Sub")) return inputs.get(0) - inputs.get(1);
        else if (method.equals("Mult")) return inputs.get(0) * inputs.get(1);
        else if (method.equals("Div")) return inputs.get(0) / inputs.get(1);
        else if (method.equals("Abs")) return Math.abs(inputs.get(0));
        else if (method.equals("Signum")) return Math.signum(inputs.get(0));
        else if (method.equals("Sin")) return Math.sin(inputs.get(0));
        else if (method.equals("Asin")) return Math.asin(inputs.get(0));
        else if (method.equals("Cos")) return Math.cos(inputs.get(0));
        else if (method.equals("Acos")) return Math.acos(inputs.get(0));
        else if (method.equals("Tan")) return Math.tan(inputs.get(0));
        else if (method.equals("Atan")) return Math.atan(inputs.get(0));
        else if (method.equals("Atan2")) return Math.atan2(inputs.get(0), inputs.get(1));
        else if (method.equals("Todeg")) return inputs.get(0) * (180.0f / Math.PI);

        else if (method.equals("And")) return ((inputs.get(0) != 0) && (inputs.get(1) != 0)) ? 1 : 0;
        else if (method.equals("Or")) return ((inputs.get(0) != 0) || (inputs.get(1) != 0)) ? 1 : 0;

        else if (method.equals("Lt")) return (inputs.get(0) < inputs.get(1)) ? 1 : 0;
        else if (method.equals("Le")) return (inputs.get(0) <= inputs.get(1)) ? 1 : 0;
        else if (method.equals("Gt")) return (inputs.get(0) > inputs.get(1)) ? 1 : 0;
        else if (method.equals("Ge")) return (inputs.get(0) >= inputs.get(1)) ? 1 : 0;

        else if (method.contains("Pow")) return Math.pow(inputs.get(0), inputs.get(1));
        else if (method.contains("Exp")) return Math.exp(inputs.get(0));
        else if (method.contains("Log")) {
            double logBase = Double.parseDouble(method.substring(3));
            return Math.log(inputs.get(0)) / Math.log(logBase);
        }

        System.out.println("EQUATION NOT DEFINED IN STATE ACTION TUPLE CLASS: " + method);
        return -1.0;
    }
}
