// STAR-CCM+ macro: macro_images.java
// Written by STAR-CCM+ 12.04.010
package macro;

import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.vis.*;
import java.net.*;
import java.io.*;

import java.util.concurrent.TimeUnit;

public class macro_path extends StarMacro {

  String caseevaluate = "STARCCMexternalfiles_2020-02-26_14-15-19";

  double xA = 0.0;
  double yA = 0.0;
  double xB = -0.02;
  double yB = 0.0;

  String path = "/scratch/daep/s.berger/ml/cfdcouplingcode/cfd/starccm/actions/" + caseevaluate + "/";

  String tableactions = caseevaluate + "bestactions.csv";
  String Tendfile = caseevaluate + "tmax.txt";
  public void execute() {
    enableupdatescene();
    wait1500();
    readtable(); //read the table of actions
    wait1500();
    tmaxchange(); //change the maximum physical time
    wait1500();
    vorticityfolder(); //save in the corresponding folder the images
    wait1500();
    ABpointsupdate(); //change the values of the A and B points
    wait1500();
    //vorticityimagewindows(); //change the view size of the vorticity figure
    //TimeUnit.MILLISECONDS.sleep(500);
    runcase(); //run the evaluation
  }

  private void wait1500() {
    try {
    Thread.sleep(1500);                 //1500 milliseconds is one second.
    } catch(InterruptedException ex) {
    Thread.currentThread().interrupt();
    }
  }

  private void readtable() {

    Simulation simulation_0 = 
      getActiveSimulation();

    FileTable fileTable_0 = 
      ((FileTable) simulation_0.getTableManager().getTable("ActionsTable"));

    fileTable_0.setFileName(resolvePath(path + tableactions));
  }

  private void tmaxchange() {
    String Tmaxstr;
    Tmaxstr = read_txt(path+Tendfile);
    double Tmax;
    
    if(Tmaxstr != null && !Tmaxstr.isEmpty()){
        Tmax = Double.parseDouble(Tmaxstr);
        Simulation simulation_0 = getActiveSimulation();
	PhysicalTimeStoppingCriterion physicalTimeStoppingCriterion_0 = 
      ((PhysicalTimeStoppingCriterion) simulation_0.getSolverStoppingCriterionManager().getSolverStoppingCriterion("Maximum Physical Time"));
	physicalTimeStoppingCriterion_0.getMaximumTime().setValue(Tmax);		
	}

  }

  private void vorticityfolder() {

    Simulation simulation_0 = getActiveSimulation();

    Scene scene_0 = 
      simulation_0.getSceneManager().getScene("vorticity");

    SceneUpdate sceneUpdate_1 = 
      scene_0.getSceneUpdate();

    //sceneUpdate_1.setAnimationFilePath(resolvePath(path + caseevaluate));
    sceneUpdate_1.setAnimationFilePath(resolvePath(path));

  }


  private void ABpointsupdate() {

    Simulation simulation_0 = getActiveSimulation();

    PointPart pointPart_0 = 
      ((PointPart) simulation_0.getPartManager().getObject("A"));

    Coordinate coordinate_0 = 
      pointPart_0.getPointCoordinate();

    Units units_0 = 
      ((Units) simulation_0.getUnitsManager().getObject("m"));

    coordinate_0.setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {xA, yA, 0.01}));

    PointPart pointPart_1 = 
      ((PointPart) simulation_0.getPartManager().getObject("B"));

    Coordinate coordinate_1 = 
      pointPart_1.getPointCoordinate();

    coordinate_1.setCoordinate(units_0, units_0, units_0, new DoubleVector(new double[] {xB, yB, 0.01}));
  }


  private void vorticityimagewindows() {

    Simulation simulation_0 = getActiveSimulation();

    Scene scene_0 = 
      simulation_0.getSceneManager().getScene("vorticity");

    CurrentView currentView_0 = scene_0.getCurrentView();

    currentView_0.setInput(new DoubleVector(new double[] {-0.008919085610215155, 5.922163744048059E-5, 0.009997956499888438}), new DoubleVector(new double[] {-0.008919085610215155, 5.922163744048059E-5, 4.296768363093393}), new DoubleVector(new double[] {0.0, 1.0, 0.0}), 0.0064963267267440766, 1);
  }


  private void runcase() {

    Simulation simulation_0 = getActiveSimulation();

    Solution solution_0 = simulation_0.getSolution();

    solution_0.clearSolution();

    solution_0.initializeSolution();

    Scene scene_0 = 
      simulation_0.getSceneManager().getScene("vorticity");

    scene_0.printAndWait(resolvePath(path + "vorticity__00000.png"), 1, 1600, 600, true, true);

    simulation_0.getSimulationIterator().run();
  }


  //READ .txt file to provide information
  private String read_txt(String filename){
    String valuestr = "";
      try{
	File f = new File(resolvePath(filename));
	Scanner scanner = new Scanner(f);
	scanner.useDelimiter(";");
	while(scanner.hasNext()){
	valuestr = scanner.next();
	}
        scanner.close();
	} catch(Exception e){}
  return valuestr;
  }

  private void disableupdatescene() {
    Simulation simulation_0 = getActiveSimulation();
    Scene scene_0 = simulation_0.getSceneManager().getScene("vorticity");
    SceneUpdate sceneUpdate_0 = scene_0.getSceneUpdate();
    sceneUpdate_0.setEnabled(false);
    sceneUpdate_0.setSaveAnimation(false);
  }

  private void enableupdatescene() {
    Simulation simulation_0 = getActiveSimulation();
    Scene scene_0 = simulation_0.getSceneManager().getScene("vorticity");
    SceneUpdate sceneUpdate_0 = scene_0.getSceneUpdate();
    sceneUpdate_0.setEnabled(true);
    sceneUpdate_0.setSaveAnimation(true);
  }
}
