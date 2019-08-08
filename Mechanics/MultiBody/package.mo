within ModelicaCI.Mechanics;
package MultiBody "Library to model 3-dimensional mechanical systems"
  extends Modelica.Icons.Package;

import SI = Modelica.SIunits;
import Cv = Modelica.SIunits.Conversions;
import C = Modelica.Constants;


annotation (
  Documentation(info="<html></html>"), Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}), graphics={
        Polygon(
          points={{-58,76},{6,76},{-26,50},{-58,76}},
          lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-26,50},{28,-50}}),
        Ellipse(
          extent={{-4,-14},{60,-78}},
          lineColor={135,135,135},
          fillPattern=FillPattern.Sphere,
          fillColor={255,255,255})}));
end MultiBody;
