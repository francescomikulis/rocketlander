package net.sf.openrocket.gui.main;

import java.awt.*;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.FlavorEvent;
import java.awt.datatransfer.FlavorListener;
import java.awt.datatransfer.StringSelection;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.io.IOException;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.Comparator;

import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.text.DefaultEditorKit;

import net.sf.openrocket.gui.simulation.MDPDefinitionEditDialog;
import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.RLModel;
import net.sf.openrocket.simulation.extension.impl.RLObjectFileStore;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import net.miginfocom.swing.MigLayout;
import net.sf.openrocket.aerodynamics.Warning;
import net.sf.openrocket.aerodynamics.WarningSet;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.document.Simulation.Status;
import net.sf.openrocket.document.events.DocumentChangeEvent;
import net.sf.openrocket.document.events.DocumentChangeListener;
import net.sf.openrocket.document.events.SimulationChangeEvent;
import net.sf.openrocket.formatting.RocketDescriptor;
import net.sf.openrocket.gui.adaptors.Column;
import net.sf.openrocket.gui.adaptors.ColumnTable;
import net.sf.openrocket.gui.adaptors.ColumnTableModel;
import net.sf.openrocket.gui.adaptors.ColumnTableRowSorter;
import net.sf.openrocket.gui.adaptors.ValueColumn;
import net.sf.openrocket.gui.components.StyledLabel;
import net.sf.openrocket.gui.simulation.SimulationRunDialog;
import net.sf.openrocket.gui.simulation.SimulationWarningDialog;
import net.sf.openrocket.gui.util.Icons;
import net.sf.openrocket.l10n.Translator;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.rocketcomponent.FlightConfigurationId;
import net.sf.openrocket.rocketcomponent.ComponentChangeEvent;
import net.sf.openrocket.rocketcomponent.ComponentChangeListener;
import net.sf.openrocket.simulation.FlightData;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.startup.Preferences;
import net.sf.openrocket.unit.UnitGroup;
import net.sf.openrocket.util.AlphanumComparator;

import static net.sf.openrocket.document.OpenRocketDocument.SIMULATION_NAME_PREFIX;
import static net.sf.openrocket.gui.util.SwingPreferences.getMaxThreadCount;

@SuppressWarnings("serial")
public class RLPanel extends JPanel {

    private static final Logger log = LoggerFactory.getLogger(RLPanel.class);
    private static final Translator trans = Application.getTranslator();


    private static final Color WARNING_COLOR = Color.RED;
    private static final String WARNING_TEXT = "\uFF01"; // Fullwidth exclamation mark

    private static final Color OK_COLOR = new Color(60, 150, 0);
    private static final String OK_TEXT = "\u2714"; // Heavy check mark


    private RocketDescriptor descriptor = Application.getInjector().getInstance(RocketDescriptor.class);


    private final OpenRocketDocument document;

    private final ColumnTableModel simulationTableModel;
    private final JTable simulationTable;

    private final JButton editButton;
    private final JButton resetModelButton;
    private final JButton deleteButton;

    public RLPanel(OpenRocketDocument doc) {
        super(new MigLayout("fill", "[grow][][][][][][grow]"));

        this.document = doc;



        ////////  The simulation action buttons

        //// New MDPDefinition
        {
            JButton button = new JButton("New MDP Definition");
            //// Add a new simulation
            button.setToolTipText("Create a new MDP Definition");
            button.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    Simulation sim = new Simulation(document.getRocket());
                    sim.setName(MDPDefinition.toJsonString(ModelBaseImplementation.getLanderDefinition()));

                    document.addSimulation(sim);
                    simulationTableModel.fireTableDataChanged();
                    simulationTable.clearSelection();

                    openDialog(false, sim);
                }
            });
            this.add(button, "skip 1, gapright para");
        }

        //// Edit simulation button
        editButton = new JButton("Edit MDP Definition");
        //// Edit the selected simulation
        editButton.setToolTipText("Modify an MDP Definition");
        editButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = simulationTable.getSelectedRows();
                if (selection.length != 1) {
                    return;
                }

                selection[0] = simulationTable.convertRowIndexToModel(selection[0]);
                Simulation[] sims = new Simulation[selection.length];
                sims[0] = document.getSimulation(selection[0]);

                openDialog(false, sims);
            }
        });
        this.add(editButton, "gapright para");

        // MODIFIED CODE HERE

        //// Reset the stateActionValueFunction
        resetModelButton = new JButton(trans.get("simpanel.but.resetmodel"));
        //// Re-run the selected simulations
        resetModelButton.setToolTipText(trans.get("simpanel.but.ttip.resetmodel"));
        resetModelButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                JPanel panel = new JPanel(new MigLayout());
                int ret = JOptionPane.showConfirmDialog(RLPanel.this,
                        new Object[] {
                                "Are you sure you want to reset the model?",
                                "<html><i>This operation cannot be undone.</i>",
                                "",
                                panel },
                        "Reset Model",
                        JOptionPane.OK_CANCEL_OPTION,
                        JOptionPane.WARNING_MESSAGE);
                if (ret != JOptionPane.OK_OPTION)
                    return;
                RLModel.getInstance().resetValueFunctionTable();
            }
        });
        this.add(resetModelButton, "gapright para");

        //// Delete simulations button
        deleteButton = new JButton("Delete MDP Definition");
        //// Delete the selected simulations
        deleteButton.setToolTipText("Please be careful");
        deleteButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] selection = simulationTable.getSelectedRows();
                if (selection.length == 0) {
                    return;
                }
                // Verify deletion
                boolean verify = Application.getPreferences().getBoolean(Preferences.CONFIRM_DELETE_SIMULATION, true);
                if (verify) {

                    JPanel panel = new JPanel(new MigLayout());
                    //// Do not ask me again
                    JCheckBox dontAsk = new JCheckBox(trans.get("simpanel.checkbox.donotask"));
                    panel.add(dontAsk, "wrap");
                    //// You can change the default operation in the preferences.
                    panel.add(new StyledLabel(trans.get("simpanel.lbl.defpref"), -2));

                    int ret = JOptionPane.showConfirmDialog(RLPanel.this,
                            new Object[] {
                                    //// Delete the selected simulations?
                                    trans.get("simpanel.dlg.lbl.DeleteSim1"),
                                    //// <html><i>This operation cannot be undone.</i>
                                    trans.get("simpanel.dlg.lbl.DeleteSim2"),
                                    "",
                                    panel },
                            //// Delete simulations
                            trans.get("simpanel.dlg.lbl.DeleteSim3"),
                            JOptionPane.OK_CANCEL_OPTION,
                            JOptionPane.WARNING_MESSAGE);
                    if (ret != JOptionPane.OK_OPTION)
                        return;

                    if (dontAsk.isSelected()) {
                        Application.getPreferences().putBoolean(Preferences.CONFIRM_DELETE_SIMULATION, false);
                    }
                }

                // Delete simulations
                for (int i = 0; i < selection.length; i++) {
                    selection[i] = simulationTable.convertRowIndexToModel(selection[i]);
                }
                Arrays.sort(selection);
                for (int i = selection.length - 1; i >= 0; i--) {
                    document.removeSimulation(selection[i]);
                }
                simulationTableModel.fireTableDataChanged();
            }
        });
        this.add(deleteButton, "wrap para");



        ////////  The simulation table

        simulationTableModel = new ColumnTableModel(

                //// Simulation name
                //// Name
                new Column(trans.get("simpanel.col.Name")) {
                    @Override
                    public Object getValueAt(int row) {
                        if (!displaySimulation(row))
                            return null;

                        return document.getSimulation(row).getName();
                    }

                    @Override
                    public int getDefaultWidth() {
                        return 1000;
                    }

                    @Override
                    public Comparator<String> getComparator() {
                        return new AlphanumComparator();
                    }
                }
        ) {

            private static final long serialVersionUID = 8686456963492628476L;

            @Override
            public int getRowCount() {
                return document.getSimulationCount();
            }
        };

        // Override processKeyBinding so that the JTable does not catch
        // key bindings used in menu accelerators
        simulationTable = new ColumnTable(simulationTableModel) {

            private static final long serialVersionUID = -5799340181229735630L;

            @Override
            protected boolean processKeyBinding(KeyStroke ks,
                                                KeyEvent e,
                                                int condition,
                                                boolean pressed) {
                return false;
            }
        };
        ColumnTableRowSorter simulationTableSorter = new ColumnTableRowSorter(simulationTableModel);
        simulationTable.setRowSorter(simulationTableSorter);
        simulationTable.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        simulationTable.setDefaultRenderer(Object.class, new JLabelRenderer());
        simulationTableModel.setColumnWidths(simulationTable.getColumnModel());

        // Mouse listener to act on double-clicks
        simulationTable.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1 && e.getClickCount() == 2) {
                    int selectedRow = simulationTable.getSelectedRow();
                    if (selectedRow < 0) {
                        return;
                    }
                    int selected = simulationTable.convertRowIndexToModel(selectedRow);

                    int column = simulationTable.columnAtPoint(e.getPoint());
                    if (column == 0) {
                        SimulationWarningDialog.showWarningDialog(RLPanel.this, document.getSimulations().get(selected));
                    } else {

                        simulationTable.clearSelection();
                        simulationTable.addRowSelectionInterval(selectedRow, selectedRow);

                        openDialog(document.getSimulations().get(selected));
                    }
                } else if (e.getButton() == MouseEvent.BUTTON3 && e.getClickCount() == 1){

                } else {
                    updateButtonStates();
                }
            }
        });

        document.addDocumentChangeListener(new DocumentChangeListener() {
            @Override
            public void documentChanged(DocumentChangeEvent event) {
                if (!(event instanceof SimulationChangeEvent))
                    return;
                simulationTableModel.fireTableDataChanged();
            }
        });




        // Fire table change event when the rocket changes
        document.getRocket().addComponentChangeListener(new ComponentChangeListener() {
            @Override
            public void componentChanged(ComponentChangeEvent e) {
                fireMaintainSelection();
            }
        });


        JScrollPane scrollpane = new JScrollPane(simulationTable);
        this.add(scrollpane, "spanx, grow, wrap rel");

        updateButtonStates();
    }

    private boolean displaySimulation(int row) {
        if (row < 0 || row >= document.getSimulationCount()) return false;
        if (document.getSimulation(row).getName().contains("{")) return true;
        return false;
    }

    public int getRealSimulationCount() {
        int realNumber = 0;
        for (int i = 0; i < document.getSimulationCount(); i++) {
            if (displaySimulation(i))
                realNumber++;
        }
        return realNumber;
    }

    private void updateButtonStates() {
        int[] selection = simulationTable.getSelectedRows();
        if (selection.length == 0) {
            editButton.setEnabled(false);
            deleteButton.setEnabled(false);
        } else {
            if (selection.length > 1) {
            } else {
            }
            editButton.setEnabled(true);
            deleteButton.setEnabled(true);
        }

    }

    public ListSelectionModel getSimulationListSelectionModel() {
        return simulationTable.getSelectionModel();
    }

    private void openDialog(boolean plotMode, final Simulation... sims) {
        MDPDefinitionEditDialog d = new MDPDefinitionEditDialog(SwingUtilities.getWindowAncestor(this), document, sims);
        d.setVisible(true);
        fireMaintainSelection();
    }

    private void openDialog(final Simulation sim) {
        boolean plotMode = false;
        if (sim.hasSimulationData() && (sim.getStatus() == Simulation.Status.UPTODATE || sim.getStatus() == Simulation.Status.EXTERNAL)) {
            plotMode = true;
        }
        openDialog(plotMode, sim);
    }

    private void fireMaintainSelection() {
        int[] selection = simulationTable.getSelectedRows();
        simulationTableModel.fireTableDataChanged();
        for (int row : selection) {
            if (row >= simulationTableModel.getRowCount())
                break;
            simulationTable.addRowSelectionInterval(row, row);
        }
    }

    public static class CellTransferable implements Transferable {

        public static final DataFlavor CELL_DATA_FLAVOR = new DataFlavor(Object.class, "application/x-cell-value");

        private Object cellValue;

        public CellTransferable(Object cellValue) {
            this.cellValue = cellValue;
        }

        @Override
        public DataFlavor[] getTransferDataFlavors() {
            return new DataFlavor[]{CELL_DATA_FLAVOR};
        }

        @Override
        public boolean isDataFlavorSupported(DataFlavor flavor) {
            return CELL_DATA_FLAVOR.equals(flavor);
        }

        @Override
        public Object getTransferData(DataFlavor flavor) throws UnsupportedFlavorException, IOException {
            if (!isDataFlavorSupported(flavor)) {
                throw new UnsupportedFlavorException(flavor);
            }
            return cellValue;
        }

    }

    private class JLabelRenderer extends DefaultTableCellRenderer {

        @Override
        public Component getTableCellRendererComponent(JTable table,
                                                       Object value, boolean isSelected, boolean hasFocus, int row,
                                                       int column) {

            if (!displaySimulation(row))
                return super.getTableCellRendererComponent(table, value,
                        isSelected, hasFocus, row, column);

            row = table.getRowSorter().convertRowIndexToModel(row);

            // A JLabel is self-contained and has set its own tool tip
            if (value instanceof JLabel) {
                JLabel label = (JLabel) value;
                if (isSelected)
                    label.setBackground(table.getSelectionBackground());
                else
                    label.setBackground(table.getBackground());
                label.setOpaque(true);
                return label;
            }

            return super.getTableCellRendererComponent(table, value,
                    isSelected, hasFocus, row, column);
        }
    }
}
