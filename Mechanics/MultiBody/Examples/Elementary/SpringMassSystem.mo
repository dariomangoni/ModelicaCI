within ModelicaCI.Mechanics.MultiBody.Examples.Elementary;
model SpringMassSystem "Mass attached with a spring to the world frame"
  extends Modelica.Icons.Example;
  parameter Boolean animation=true "= true, if animation shall be enabled";
  inner Modelica.Mechanics.MultiBody.World world annotation (Placement(
        transformation(extent={{-80,20},{-60,40}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic p1(useAxisFlange=true,
    n={0,-1,0},
    animation=animation,
    boxWidth=0.05,
    s(fixed=true, start=0.1),
    v(fixed=true)) annotation (Placement(transformation(
        origin={-20,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Translational.Components.Spring spring1(
                                                  c=30, s_rel0=0.1)
    annotation (Placement(transformation(
        origin={10,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(
    m=1,
    sphereDiameter=0.2,
    animation=animation,
    r_CM={0,0,0}) annotation (Placement(transformation(
        origin={-20,-50},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation bar1(animation=animation, r={0.3,0,0})
    annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation bar2(animation=animation, r={0.3,0,0})
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Mechanics.MultiBody.Parts.Body body2(
    m=1,
    sphereDiameter=0.2,
    animation=animation,
    r_CM={0,0,0}) annotation (Placement(transformation(
        origin={40,-50},
        extent={{-10,10},{10,-10}},
        rotation=270)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic p2(useAxisFlange=true,
    n={0,-1,0},
    animation=animation,
    boxWidth=0.05,
    stateSelect=StateSelect.always,
    s(fixed=true, start=0.1),
    v(fixed=true)) annotation (Placement(transformation(
        origin={40,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.MultiBody.Forces.Spring spring2(
    c=30,
    s_unstretched=0.1,
    width=0.1) annotation (Placement(transformation(
        origin={60,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
equation
  connect(body1.frame_a, p1.frame_b)
    annotation (Line(
      points={{-20,-40},{-20,-35},{-20,-35},{-20,-30},{-20,-20},{-20,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(world.frame_b, bar1.frame_a)
    annotation (Line(
      points={{-60,30},{-50,30}},
      color={95,95,95},
      thickness=0.5));
  connect(bar1.frame_b, p1.frame_a) annotation (Line(
      points={{-30,30},{-20,30},{-20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(spring1.flange_b, p1.axis) annotation (Line(points={{10,-20},{-8,-20},{-8,-18},{-14,-18}}, color={0,127,0}));
  connect(bar1.frame_b, bar2.frame_a)
    annotation (Line(
      points={{-30,30},{0,30}},
      color={95,95,95},
      thickness=0.5));
  connect(bar2.frame_b, p2.frame_a)
    annotation (Line(
      points={{20,30},{40,30},{40,0}},
      color={95,95,95},
      thickness=0.5));
  connect(p2.frame_b, body2.frame_a)
    annotation (Line(
      points={{40,-20},{40,-40}},
      color={95,95,95},
      thickness=0.5));
  connect(bar2.frame_b, spring2.frame_a)
    annotation (Line(
      points={{20,30},{40,30},{40,10},{60,10},{60,0}},
      color={95,95,95},
      thickness=0.5));
  connect(body2.frame_a, spring2.frame_b) annotation (Line(
      points={{40,-40},{40,-40},{40,-30},{60,-30},{60,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(spring1.flange_a, p1.support) annotation (Line(points={{10,0},{-8,0},{-8,-6},{-14,-6}}, color={0,127,0}));
  annotation (
    experiment(StopTime=5),
    Documentation(info="<html>
<p>
This example shows the two different ways how force laws
can be utilized:
</p>
<ul>
<li>In the left system a body is attached via a prismatic
    joint to the world frame. The prismatic joint has two
    1-dimensional translational flanges (called \"support\" and \"axis\")
    that allows to connect elements from the Modelica.Mechanics.Translational
    library between the support and the axis connector. The effect is
    that the force generated by the 1-dimensional elements acts as driving
    force in the axis of the prismatic joint. In the example a simple
    spring is used.<br>
    The advantage of this approach is that the many elements from the
    Translational library can be easily used here and that this implementation
    is usually more efficient as when using 3-dimensional springs.</li>
<li>In the right system the same model is defined. The difference is
    that a 3-dimensional spring from the Modelica.Mechanics.MultiBody.Forces library is used.
    This has the advantage to get a nice animation of the force component.</li>
</ul>

<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/SpringMassSystem.png\"
alt=\"model Examples.Elementary.SpringMassSystem\">
</html>"));
end SpringMassSystem;
