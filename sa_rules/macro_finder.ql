/**
 * @id cpp/sensor_parm_finder
 * @name Sensor Param Finder
 * @description Finds Sensor Functions 
 * @tags macro_custom 
 */

import cpp

// Find the sensor functions inside Parameters.cpp inside the AP_Periph folder
from Function f, Parameter p, MacroInvocation m
where
  // f.getFile().getAbsolutePath().toString().indexOf("AP_Periph/Parameters.cpp") >= 0
  // f.getFile().toString().matches("%Parameters.cpp")
  m.getFile().toString().matches("%Parameters.cpp") and
  m.getMacroName().matches("GOBJECT")
select m, m.getStmt(), m.getExpr()