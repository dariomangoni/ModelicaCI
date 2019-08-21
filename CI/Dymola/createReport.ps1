# libraryLocation = "D:/ActivateLibraries/ModelicaCode/ModelicaCI/package.mo";

$experimentNameList = @("ModelicaCI.Mechanics.MultiBody.Examples.Constraints.PrismaticConstraint",
  "ModelicaCI.Mechanics.MultiBody.Examples.Constraints.RevoluteConstraint",
  "ModelicaCI.Mechanics.MultiBody.Examples.Constraints.SphericalConstraint",
  "ModelicaCI.Mechanics.MultiBody.Examples.Constraints.UniversalConstraint",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.DoublePendulum",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.DoublePendulumInitTip",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.ForceAndTorque",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.FreeBody",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.HeatLosses",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.InitSpringConstant",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.LineForceWithTwoMasses",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.Pendulum",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.PendulumWithSpringDamper",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.PointGravity",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.PointGravityWithPointMasses",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.PointGravityWithPointMasses2",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.RollingWheel",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.RollingWheelSetDriving",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.RollingWheelSetPulling",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.SpringDamperSystem",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.SpringMassSystem",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.SpringWithMass",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.Surfaces",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.ThreeSprings",
  "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.UserDefinedGravityField",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Engine1a",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Engine1b",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Engine1b_analytic",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.EngineV6",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.EngineV6_analytic",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Fourbar1",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Fourbar2",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.Fourbar_analytic",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.PlanarFourbar",
  "ModelicaCI.Mechanics.MultiBody.Examples.Loops.PlanarLoops_analytic",
  "ModelicaCI.Mechanics.MultiBody.Examples.Rotational3DEffects.ActuatedDrive",
  "ModelicaCI.Mechanics.MultiBody.Examples.Rotational3DEffects.BevelGear1D",
  "ModelicaCI.Mechanics.MultiBody.Examples.Rotational3DEffects.GearConstraint",
  "ModelicaCI.Mechanics.MultiBody.Examples.Rotational3DEffects.GyroscopicEffects",
  "ModelicaCI.Mechanics.MultiBody.Examples.Rotational3DEffects.MovingActuatedDrive",
  "ModelicaCI.Mechanics.MultiBody.Examples.Systems.RobotR3.fullRobot",
  "ModelicaCI.Mechanics.MultiBody.Examples.Systems.RobotR3.oneAxis");

# $experimentNameList = @("ModelicaCI.Mechanics.MultiBody.Examples.Elementary.FreeBody",
#   "ModelicaCI.Mechanics.MultiBody.Examples.Elementary.ForceAndTorque");

$tempWorkingDirectory = "D:/ActivateLibraries/ModelicaCode/ModelicaCI_build/Dymola";

$experimentNum = $experimentNameList.Length;
$cpuTime = @(9999)*$experimentNum;
For ($i=0; $i -lt $experimentNum; $i++) {
    $dymolaLogFilePath = $tempWorkingDirectory+"\"+$experimentNameList[$i]+"\dslog.txt";

    $logContent = Get-Content $dymolaLogFilePath
    $res = Select-String -InputObject $logContent -Pattern 'CPU\-time for integration\s*\: \s*([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)\s*seconds'
    $cpuTime[$i] = $res.Matches[0].Groups[1].Value;
}


$reportFile = $tempWorkingDirectory+"/report.html";
$reportContent = "<!DOCTYPE html>
<html>
<head>
<style>
  body { font-family: Droid Serif,serif; font-size: 16px; line-height: 24px; width: 800px; margin: 24px auto }
  p, dt, pre, blockquote, ol, ul, table, hr { margin-top: 24px; margin-bottom: 0 }
  img { margin-top: 24px }
  img.icon, embed.icon { float: right; margin: 0 0 24px 24px; border: 0 }
  h1, h2, h3, h4, h5, th { font-family: Droid Sans,sans-serif }
  h1 { font-size: 26px; line-height: 26px; margin: 18px 0 0 0 }h2 { font-size: 21px; line-height: 24px; margin: 26px 0 -2px 0 }h3 { font-size: 18px; line-height: 24px; margin: 27px 0 -3px 0 }h4 { font-size: 16px; line-height: 24px; margin: 28px 0 -4px 0; font-style: italic }h1 .subtitle, h2 .subtitle { font-size: 16px; font-style: italic }
  h1 br, h2 br { margin-bottom: 10px }
  p.interface { background-color: #EEE; padding: 20px; border: 1px solid #CCE; border-radius: 14px }
  code, pre, p.interface { font-family: Droid Sans Mono,monospace; font-size: 14px; line-height: 20px }
  li, dd, li p, dd p, li dt, dd dt, li pre, dd pre, li blockquote, dd blockquote, li table, dd table { margin-top: 11px; margin-bottom: 11px }
  dt + dt, dd, ul ul { margin-top: 0 }
  blockquote pre, blockquote blockquote { margin-top: 0; margin-bottom: 0 }
  ul ul li { margin-top:5px; margin-bottom:6px }
  td, th { vertical-align: center; font-size: 14px; line-height: 20px }
  td.center, th.center { text-align: center; vertical-align: center; font-size: 14px; line-height: 20px }
  th { background-color: #EEE }
  td p, th p { margin-top: 10px }
  td code, th code { font-size: 13px }
  hr { border: 0; border-bottom: 1px dotted darkred; clear: right }
</style>
    
</head>
<body>
";

$reportContent += [Array] "<table class=`"ReportTimeTable`">
";
$reportContent += [Array] "  <tr><th>Experiment</th><th>Simulation Time [s]</th></tr>
";

For ($i=0; $i -lt $experimentNum; $i++) {
  Write-Host "Experiment :"$experimentNameList[$i]" took "$cpuTime[$i]"s to run";
  $reportContent += [Array] "  <tr><td>"+$experimentNameList[$i]+"</td><td class=`"center`">"+$cpuTime[$i]+"</td></tr>
  ";
}
$reportContent += [Array] "</table>";

$reportContent += [Array] "</body>
</html>";



$reportContent | Out-File $reportFile
Invoke-Expression $reportFile
