within ModelicaCI.Mechanics.MultiBody.Examples.Systems;
package RobotR3
  "Library to demonstrate robot system models based on the Manutec r3 robot"
  extends Modelica.Icons.ExamplesPackage;

  model oneAxis
    "Model of one axis of robot (controller, motor, gearbox) with simple load"
    extends ModelicaCI.Interfaces.ExamplesOutput;

    extends Modelica.Icons.Example;
    parameter SI.Mass mLoad(min=0)=15 "Mass of load";
    parameter Real kp=5 "Gain of position controller of axis 2";
    parameter Real ks=0.5 "Gain of speed controller of axis 2";
    parameter SI.Time Ts=0.05
      "Time constant of integrator of speed controller of axis 2";
    parameter Real startAngle(unit="deg") = 0 "Start angle of axis 2";
    parameter Real endAngle(unit="deg") = 120 "End angle of axis 2";

    parameter SI.Time swingTime=0.5
      "Additional time after reference motion is in rest before simulation is stopped";
    parameter SI.AngularVelocity refSpeedMax=3 "Maximum reference speed";
    parameter SI.AngularAcceleration refAccMax=10
      "Maximum reference acceleration";

    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType1 axis(
      w=5500,
      ratio=210,
      c=8,
      cd=0.01,
      Rv0=0.5,
      Rv1=(0.1/130),
      kp=kp,
      ks=ks,
      Ts=Ts) annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Modelica.Mechanics.Rotational.Components.Inertia load(J=1.3*mLoad)
      annotation (Placement(transformation(extent={{60,0},{80,20}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.PathPlanning1 pathPlanning(
      swingTime=swingTime,
      angleBegDeg=startAngle,
      angleEndDeg=endAngle,
      speedMax=refSpeedMax,
      accMax=refAccMax) annotation (Placement(transformation(extent={{-60,0},
              {-40,20}})));
  protected
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.ControlBus controlBus annotation (Placement(transformation(
            extent={{-32,10},{8,50}})));
  equation
    load.flange_b.phi = outVal;
    connect(axis.flange, load.flange_a)
      annotation (Line(
        points={{40,10},{60,10}},
        color={128,128,128},
        thickness=0.5));
    connect(pathPlanning.controlBus, controlBus) annotation (Line(
        points={{-40,10},{-15,10},{-15,28},{-12,28},{-12,30}},
        color={255,204,51},
        thickness=0.5));
    connect(controlBus.axisControlBus1, axis.axisControlBus) annotation (
      Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-11.9,30.1},{-11.9,29},{-9,29},{-9,10},{20,10}},
        color={255,204,51},
        thickness=0.5));
    annotation (
      Documentation(info="<html>
<p>
With this model one axis of the r3 robot is checked.
The mechanical structure is replaced by a simple
load inertia.
</p>
</html>"),      experiment(StopTime=1.6),
      __Dymola_Commands(file="modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/oneAxisPlot.mos"
          "Plot result"));
  end oneAxis;

  model fullRobot
    "Six degree of freedom robot with path planning, controllers, motors, brakes, gears and mechanics"
    extends ModelicaCI.Interfaces.ExamplesOutput;
    extends Modelica.Icons.Example;

    parameter SI.Mass mLoad(min=0) = 15 "Mass of load";
    parameter SI.Position rLoad[3]={0.1,0.25,0.1}
      "Distance from last flange to load mass";
    parameter SI.Acceleration g=9.81 "Gravity acceleration";
    parameter SI.Time refStartTime=0 "Start time of reference motion";
    parameter SI.Time refSwingTime=0.5
      "Additional time after reference motion is in rest before simulation is stopped";

    parameter Real startAngle1(unit="deg") = -60 "Start angle of axis 1"
      annotation (Dialog(tab="Reference", group="startAngles"));
    parameter Real startAngle2(unit="deg") = 20 "Start angle of axis 2"
      annotation (Dialog(tab="Reference", group="startAngles"));
    parameter Real startAngle3(unit="deg") = 90 "Start angle of axis 3"
      annotation (Dialog(tab="Reference", group="startAngles"));
    parameter Real startAngle4(unit="deg") = 0 "Start angle of axis 4"
      annotation (Dialog(tab="Reference", group="startAngles"));
    parameter Real startAngle5(unit="deg") = -110 "Start angle of axis 5"
      annotation (Dialog(tab="Reference", group="startAngles"));
    parameter Real startAngle6(unit="deg") = 0 "Start angle of axis 6"
      annotation (Dialog(tab="Reference", group="startAngles"));

    parameter Real endAngle1(unit="deg") = 60 "End angle of axis 1"
      annotation (Dialog(tab="Reference", group="endAngles"));
    parameter Real endAngle2(unit="deg") = -70 "End angle of axis 2"
      annotation (Dialog(tab="Reference", group="endAngles"));
    parameter Real endAngle3(unit="deg") = -35 "End angle of axis 3"
      annotation (Dialog(tab="Reference", group="endAngles"));
    parameter Real endAngle4(unit="deg") = 45 "End angle of axis 4"
      annotation (Dialog(tab="Reference", group="endAngles"));
    parameter Real endAngle5(unit="deg") = 110 "End angle of axis 5"
      annotation (Dialog(tab="Reference", group="endAngles"));
    parameter Real endAngle6(unit="deg") = 45 "End angle of axis 6"
      annotation (Dialog(tab="Reference", group="endAngles"));

    parameter SI.AngularVelocity refSpeedMax[6]={3,1.5,5,3.1,3.1,4.1}
      "Maximum reference speeds of all joints"
      annotation (Dialog(tab="Reference", group="Limits"));
    parameter SI.AngularAcceleration refAccMax[6]={15,15,15,60,60,60}
      "Maximum reference accelerations of all joints"
      annotation (Dialog(tab="Reference", group="Limits"));

    parameter Real kp1=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 1"));
    parameter Real ks1=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 1"));
    parameter SI.Time Ts1=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 1"));
    parameter Real kp2=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 2"));
    parameter Real ks2=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 2"));
    parameter SI.Time Ts2=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 2"));
    parameter Real kp3=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 3"));
    parameter Real ks3=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 3"));
    parameter SI.Time Ts3=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 3"));
    parameter Real kp4=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 4"));
    parameter Real ks4=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 4"));
    parameter SI.Time Ts4=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 4"));
    parameter Real kp5=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 5"));
    parameter Real ks5=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 5"));
    parameter SI.Time Ts5=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 5"));
    parameter Real kp6=5 "Gain of position controller"
      annotation (Dialog(tab="Controller", group="Axis 6"));
    parameter Real ks6=0.5 "Gain of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 6"));
    parameter SI.Time Ts6=0.05
      "Time constant of integrator of speed controller"
      annotation (Dialog(tab="Controller", group="Axis 6"));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.MechanicalStructure mechanics(
      mLoad=mLoad,
      rLoad=rLoad,
      g=g) annotation (Placement(transformation(extent={{40,-30},{100,30}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.PathPlanning6
      pathPlanning(
      naxis=6,
      angleBegDeg={startAngle1,startAngle2,startAngle3,startAngle4,startAngle5,
          startAngle6},
      angleEndDeg={endAngle1,endAngle2,endAngle3,endAngle4,endAngle5,endAngle6},
      speedMax=refSpeedMax,
      accMax=refAccMax,
      startTime=refStartTime,
      swingTime=refSwingTime) annotation (Placement(transformation(extent={{0,70},{-20,90}})));

    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType1 axis1(
      w=4590,
      ratio=-105,
      c=43,
      cd=0.005,
      Rv0=0.4,
      Rv1=(0.13/160),
      kp=kp1,
      ks=ks1,
      Ts=Ts1) annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType1 axis2(
      w=5500,
      ratio=210,
      c=8,
      cd=0.01,
      Rv1=(0.1/130),
      Rv0=0.5,
      kp=kp2,
      ks=ks2,
      Ts=Ts2) annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));

    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType1 axis3(
      w=5500,
      ratio=60,
      c=58,
      cd=0.04,
      Rv0=0.7,
      Rv1=(0.2/130),
      kp=kp3,
      ks=ks3,
      Ts=Ts3) annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType2 axis4(
      k=0.2365,
      w=6250,
      D=0.55,
      J=1.6e-4,
      ratio=-99,
      Rv0=21.8,
      Rv1=9.8,
      peak=26.7/21.8,
      kp=kp4,
      ks=ks4,
      Ts=Ts4) annotation (Placement(transformation(extent={{-20,0},{0,20}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType2 axis5(
      k=0.2608,
      w=6250,
      D=0.55,
      J=1.8e-4,
      ratio=79.2,
      Rv0=30.1,
      Rv1=0.03,
      peak=39.6/30.1,
      kp=kp5,
      ks=ks5,
      Ts=Ts5) annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.AxisType2 axis6(
      k=0.0842,
      w=7400,
      D=0.27,
      J=4.3e-5,
      ratio=-99,
      Rv0=10.9,
      Rv1=3.92,
      peak=16.8/10.9,
      kp=kp6,
      ks=ks6,
      Ts=Ts6) annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  protected
    Modelica.Mechanics.MultiBody.Examples.Systems.RobotR3.Components.ControlBus controlBus
      annotation (Placement(transformation(
          origin={-80,0},
          extent={{-20,-20},{20,20}},
          rotation=90)));
  equation
    axis1.flange.phi = outVal;
    connect(axis2.flange, mechanics.axis2) annotation (Line(points={{0,-30},{20,-30},{20,-13.5},{40,-13.5}}));
    connect(axis1.flange, mechanics.axis1) annotation (Line(points={{0,-50},{30,-50},{30,-22.5},{40,-22.5}}));
    connect(axis3.flange, mechanics.axis3) annotation (Line(points={{0,-10},{10,-10},{10,-4.5},{40,-4.5}}));
    connect(axis4.flange, mechanics.axis4) annotation (Line(points={{0,10},{10,10},{10,4.5},{40,4.5}}));
    connect(axis5.flange, mechanics.axis5)
      annotation (Line(points={{0,30},{20,30},{20,13.5},{40,13.5}}));
    connect(axis6.flange, mechanics.axis6) annotation (Line(points={{0,50},{30,50},{30,22.5},{40,22.5}}));
    connect(controlBus, pathPlanning.controlBus)
                                         annotation (Line(
        points={{-80,0},{-80,80},{-20,80}},
        color={255,204,51},
        thickness=0.5));
    connect(controlBus.axisControlBus1, axis1.axisControlBus) annotation (
      Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-80.1,-4.5},{-79,-4.5},{-79,-7},{-68,-7},{-68,-50},{-20,-50}},
        color={255,204,51},
        thickness=0.5));

    connect(controlBus.axisControlBus2, axis2.axisControlBus) annotation (
      Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-79,0.1},{-79,-5},{-64,-5},{-64,-30},{-20,-30}},
        color={255,204,51},
        thickness=0.5));

    connect(controlBus.axisControlBus3, axis3.axisControlBus) annotation (Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-77,0.1},{-77,-2.5},{-60,-2.5},{-60,-10},{-20,-10}},
        color={255,204,51},
        thickness=0.5));

    connect(controlBus.axisControlBus4, axis4.axisControlBus) annotation (Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-60,0.1},{-60,10},{-20,10}},
        color={255,204,51},
        thickness=0.5));
    connect(controlBus.axisControlBus5, axis5.axisControlBus) annotation (
      Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-77,0.1},{-77,3},{-64,3},{-64,30},{-20,30}},
        color={255,204,51},
        thickness=0.5));
    connect(controlBus.axisControlBus6, axis6.axisControlBus) annotation (
      Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}), Line(
        points={{-80.1,0.1},{-79,0.1},{-79,5},{-68,5},{-68,50},{-20,50}},
        color={255,204,51},
        thickness=0.5));
    annotation (
      experiment(StopTime=2),
      __Dymola_Commands(
        file="modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/Run.mos"
          "Simulate",
        file="modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/fullRobotPlot.mos"
          "Plot result of axis 3 + animate"),
      Documentation(info="<html>
<p>
This example animates a motion of a detailed model of the robot
with predefined axes' angles over time.
For animation, CAD data is used.
Translate and simulate with the default settings
(default simulation stop time = 2&nbsp;s).
</p>
<p>
The path planning block incorporates a simulation termination condition.
Thus, the simulation can be terminated before reaching the stop time.
The condition depends on the start and end positions of the joints, and on their
reference speeds and reference accelerations.
For current settings, the termination condition should indeed be fulfilled right before the simulation stops.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Systems/r3_fullRobot.png\" alt=\"model Examples.Loops.Systems.RobotR3.fullRobot\">
</p>
</html>"));
  end fullRobot;



  annotation (
    Documentation(info="<html>
<p>
This package contains models of the robot r3 of the company Manutec.
These models are used to demonstrate in which way complex
robot models might be built up by testing first the component
models individually before composing them together.
Furthermore, it is shown how CAD data can be used
for animation.
</p>

<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Systems/robot_kr15.png\"
alt=\"model Examples.Systems.RobotR3\">

<p>
The following models are available:
</p>
<pre>
   <strong>oneAxis</strong>   Test one axis (controller, motor, gearbox).
   <strong>fullRobot</strong> Test complete robot model.
</pre>
<p>
The r3 robot is no longer manufactured. In fact the company
Manutec does no longer exist.
The parameters of this robot have been determined by measurements
in the laboratory of DLR. The measurement procedure is described in:
</p>
<pre>
   Tuerk S. (1990): Zur Modellierung der Dynamik von Robotern mit
       rotatorischen Gelenken. Fortschrittberichte VDI, Reihe 8, Nr. 211,
       VDI-Verlag 1990.
</pre>
<p>
The robot model is described in detail in
</p>
<pre>
   Otter M. (1995): Objektorientierte Modellierung mechatronischer
       Systeme am Beispiel geregelter Roboter. Dissertation,
       Fortschrittberichte VDI, Reihe 20, Nr. 147, VDI-Verlag 1995.
       This report can be downloaded as compressed postscript file
       from: <a href=\"http://www.robotic.dlr.de/Martin.Otter\">http://www.robotic.dlr.de/Martin.Otter</a>.
</pre>
<p>
The path planning is performed in a simple way by using essentially
the Modelica.Mechanics.Rotational.KinematicPTP block. A user defines
a path by start and end angle of every axis. A path is planned such
that all axes are moving as fast as possible under the given
restrictions of maximum joint speeds and maximum joint accelerations.
The actual r3 robot from Manutec had a different path planning strategy.
Today's path planning algorithms from robot companies are much
more involved.
</p>
<p>
In order to get a nice animation, CAD data from a KUKA robot
is used, since CAD data of the original r3 robot was not available.
The KUKA CAD data was derived from public data of
<a href=\"http://www.kuka-robotics.com/\">
KUKA</a>.
Since dimensions of the corresponding KUKA robot are similar but not
identical to the r3 robot, the data of the r3 robot (such as arm lengths) have been modified, such that it matches the CAD data.
</p>
<p>
In this model, a simplified P-PI cascade controller for every
axes is used. The parameters have been manually adjusted by
simulations. The original r3 controllers are more complicated.
The reason to use simplified controllers is to have a simpler demo.
</p>
</html>"));

end RobotR3;
