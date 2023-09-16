package frc.team3128.common.utility;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Wrapper for {@link Shuffleboard}
 * @since 2022 RAPID REACT
 * @author Mason Lam
 */
public class NAR_Shuffleboard {

    /**
     * Storage class for NAR_Shuffleboard
     */
    private static class entryInfo {
        
        private GenericEntry m_data;

        private Supplier<Object> m_supply;

        private SimpleWidget m_entry;

        /**
         * Creates a new entry Info
         *
         * @param entry Widget where the entry 
         * @param supply supplier updating the entry
         */
        public entryInfo(SimpleWidget entry, Supplier<Object> supply){
            m_supply = supply;
            m_entry = entry;
            m_data = entry.getEntry();
        }

        public void update() {
            if(m_supply == null) return;
            m_data.setValue(m_supply.get());
        }
    }

    private static HashMap<String, HashMap<String, entryInfo>> tabs = new HashMap<String, HashMap<String,entryInfo>>();;

    /**
   * Creates a new tab entry
   *
   * @param tabName the title of the new tab
   */
    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,entryInfo>());
    }

    /**
   * Displays a value in Shuffleboard
   *
   * @param tabName the title of the tab to select
   * @param name the name of the entry
   * @param data value to display
   * @param x -coord of the entry starting from 0
   * @param y -coord of the entry starting from 0
   * @return simple widget that can be modified
   */
    public static SimpleWidget addData(String tabName, String name, Object data, int x, int y) {
        return addData(tabName, name, data, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param supply object supplier to constantly update value
     * @param x -coord of the entry starting from 0
     * @param y -coord of the entry starting from 0
     * @return simple widget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y) {
        return addData(tabName, name, supply, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param supply object supplier to constantly update value
     * @param x -coord of the entry starting from 0
     * @param y -coord of the entry starting from 0
     * @param width -of the entry
     * @param height -of the entry
     * @return simple widget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y, int width, int height){
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        if(tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_supply = supply;
            return tabs.get(tabName).get(name).m_entry;
        }
        SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,supply.get()).withPosition(x, y).withSize(width, height);
        tabs.get(tabName).put(name, new entryInfo(entry,supply));
        return entry;
    }

    /**
   * Displays a value in Shuffleboard
   *
   * @param tabName the title of the tab to select
   * @param name the name of the entry
   * @param data value to display
   * @param x -coord of the entry starting from 0
   * @param y -coord of the entry starting from 0
   * @param width -of the entry
   * @param height -of the entry
   * @return simple widget that can be modified
   */
  public static SimpleWidget addData(String tabName, String name, Object data, int x, int y, int width, int height) {
    if(!tabs.containsKey(tabName)) create_tab(tabName);
    if (tabs.get(tabName).containsKey(name)) {
        tabs.get(tabName).get(name).m_data.setValue(data);
        return tabs.get(tabName).get(name).m_entry;
    }
    SimpleWidget entry = Shuffleboard.getTab(tabName).add(name,data).withPosition(x, y).withSize(width,height);
    tabs.get(tabName).put(name, new entryInfo(entry,null));
    return entry;
}

    /**
     * Displays complex values, like subsystems and command, works on all classes that extend sendable
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param data complex value to display
     * @param x x-coord of the entry
     * @param y y-coord of the entry
     * @return complex widget that can be modified
     */
    public static ComplexWidget addComplex(String tabName, String name, Sendable data, int x, int y) {
        try {
            return Shuffleboard.getTab(tabName).add(name, data).withPosition(x,y);
        }
        catch(Exception e) {
            return null;
        }
    }

    /**
     * Displays complex values, like subsystems and command, works on all classes that extend sendable
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param data complex value to display
     * @param x x-coord of the entry
     * @param y y-coord of the entry
     * @return complex widget that can be modified
     */
    public static ComplexWidget addComplex(String tabName, String name, Sendable data, int x, int y, int width, int height) {
        return addComplex(tabName, name, data, x, y).withSize(width,height);
    }

    /**
     * Creates a debug entry, allows user to edit variable from Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param Default starting value for the entry
     * @param x x-coord of the entry
     * @param y y-coord of the entry
     * @return DoubleSupplier containing the value in the entry
     */
    public static DoubleSupplier debug(String tabName, String name, double Default, int x, int y) {
        if(!tabs.containsKey(tabName)){
            create_tab(tabName);
        }
        SimpleWidget tab = addData(tabName, name, Default, x, y);
        return ()-> tab.getEntry().getDouble(Default);
    }

    /**
     * Creates a quick PID Tuning setup
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @param prefix String that goes before PID entry names
     * @return HashMap with keys "KF","KP","KI","KD", and "SETPOINT"
     */
    public static HashMap<String,DoubleSupplier> PID_Setup(String tabName, String prefix) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        HashMap<String,DoubleSupplier> PID = new HashMap<String,DoubleSupplier>();
        for (String i : new String[]{"KF","KP","KI","KD","SETPOINT"}) {
            GenericEntry entry = tab.add(prefix + "_" + i,0).getEntry();
            PID.put(i,()-> entry.getDouble(0));
        }
        return PID;
    }

    /**
     * Get the value from an entry
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @return Object stored in the entry
     */
    public static Object getValue(String tabName, String name){
        return tabs.get(tabName).get(name).m_data.get().getValue();
    }

    /**
     * Get the Simple Widget object from an entry
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the entry
     * @return SimpleWidget stored in the entry
     */
    public static SimpleWidget getEntry(String tabName,String name) {
        return tabs.get(tabName).get(name).m_entry;
    }

    /**
     * Updates every entry
     */
    public static void update() {
        for(String i : tabs.keySet()){
            for(String j : tabs.get(i).keySet()){
                tabs.get(i).get(j).update();
            }
        }
    }

}