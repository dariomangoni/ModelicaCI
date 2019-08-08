within ModelicaCI.Mechanics.MultiBody.Examples.Elementary;
model PointGravity "Two point masses in a point gravity field"
  extends ModelicaCI.Interfaces.ExamplesOutput;
  extends Modelica.Icons.Example;
  inner Modelica.Mechanics.MultiBody.World world(
    mue=1,
    gravitySphereDiameter=0.1,
    gravityType=Modelica.Mechanics.MultiBody.Types.GravityTypes.PointGravity)
                               annotation (Placement(transformation(extent={{
            -20,-20},{0,0}})));
  Modelica.Mechanics.MultiBody.Parts.Body body1(
    m=1,
    sphereDiameter=0.1,
    I_11=0.1,
    I_22=0.1,
    I_33=0.1,
    r_0(start={0,0.6,0}, each fixed=true),
    v_0(start={1,0,0}, each fixed=true),
    angles_fixed=true,
    w_0_fixed=true,
    r_CM={0,0,0})
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
equation
  Modelica.Math.Vectors.norm(body1.frame_a.R.w) = outVal;

    annotation (Placement(transformation(extent={{20,20},{40,40}})),
    experiment(StopTime=5));
end PointGravity;
