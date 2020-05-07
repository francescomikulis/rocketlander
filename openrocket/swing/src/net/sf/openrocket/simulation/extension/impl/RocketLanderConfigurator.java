package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.components.BasicSlider;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.simulation.extension.impl.rocketlander.RocketLander;
import net.sf.openrocket.unit.UnitGroup;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;
import org.fife.ui.rsyntaxtextarea.SyntaxConstants;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;

@Plugin
public class RocketLanderConfigurator extends AbstractSwingSimulationExtensionConfigurator<RocketLander> {
	private RSyntaxTextArea text;

	public RocketLanderConfigurator() {
		super(RocketLander.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(RocketLander extension, Simulation simulation, JPanel panel) {
		JLabel label = new JLabel("Initial conditions, format = '[min, max]'.  Angles < 1 converted from degrees to rad");
		panel.add(label, "w 75lp, wrap");

		text = new RSyntaxTextArea(extension.getRLInitialConditions(), 12, 40);
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
					extension.setRLInitialConditions(text.getText());
					text.setText(extension.getRLInitialConditions());
				} catch (Exception exc) {
					JOptionPane.showMessageDialog(panel, new Object[] {"GSON PARSING ERROR\n", exc.getMessage()});
				}
			}
		});
		panel.add(save, "tag ok");
		
		return panel;
	}
	
}
