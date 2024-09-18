/**
 * @id cpp/sensor_parm_finder
 * @name Sensor Param Finder
 * @description Finds Sensor Functions 
 * @tags macro_custom 
 */

import cpp

from SwitchCase sc, SwitchStmt ss, MacroInvocation mi
where
  sc.getExpr().toString().regexpMatch("(?i).*MAV.*")
  and sc.getSwitchStmt() = ss
  and (ss.getExpr().toString().regexpMatch("(?i).*command.*") or
  ss.getExpr().toString().regexpMatch("(?i).*id.*"))
  // BREAKPOINT: Check what can allow us to actualy find the macro that affects the case
  // and ss.isAffectedByMacro() and mi.getMacroName() = ss.getExpr().toString()
  // Check the case is affected by a Macro and print the macro name
select sc.getExpr(), ss.getExpr()
