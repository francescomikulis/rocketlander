package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.simulation.extension.SimulationExtension;
import net.sf.openrocket.simulation.extension.impl.initialconditions.AbstractGenericInitialConditionsExtension;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;


public abstract class GenericInitialConditionsConfigurator<E extends SimulationExtension> extends AbstractSwingSimulationExtensionConfigurator<E> {
    private RSyntaxTextArea text;

    protected GenericInitialConditionsConfigurator(Class<E> extensionClass) {
        super(extensionClass);
    }

    protected final JComponent addInitialConditionsToComponent(AbstractGenericInitialConditionsExtension extension, Simulation simulation, JPanel panel) {
        JLabel label = new JLabel("Initial conditions, format = '[min, max]'.  Angles < 1 converted from degrees to rad");
        panel.add(label, "w 75lp, wrap");

        //// Reset button
        JButton reset = new JButton("Reset To Default");
        reset.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                extension.resetInitialConditionsToDefault();
                text.setText(extension.getInitialConditions());
            }
        });
        panel.add(reset, "wrap");

        text = new RSyntaxTextArea(extension.getInitialConditions(), 14, 40);
        text.setEditable(true);
        text.setCaretPosition(0);
        text.setCodeFoldingEnabled(true);
        text.setLineWrap(true);
        text.setWrapStyleWord(true);
        text.addFocusListener(new FocusListener() {
            @Override
            public void focusGained(FocusEvent event) { }
            @Override
            public void focusLost(FocusEvent event) { }
        });
        JScrollPane scroll = new JScrollPane(text);
        panel.add(scroll, "spanx, grow, wrap para");

        //// Save button
        JButton save = new JButton("Save");
        save.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                try {
                    extension.setInitialConditions(text.getText());
                    text.setText(extension.getInitialConditions());
                } catch (Exception exc) {
                    JOptionPane.showMessageDialog(panel, new Object[] {"GSON PARSING ERROR\n", exc.getMessage()});
                }
            }
        });
        panel.add(save, "tag ok");

        return panel;
    }

}
