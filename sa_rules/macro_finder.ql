/**
 * @id cpp/sensor_parm_finder
 * @name Sensor Param Finder
 * @description Finds Sensor Functions 
 * @tags macro_custom 
 */

import cpp

// Find all the if directives that are ENABLEd :)
// from PreprocessorIf preprocessorIf

// where
//     preprocessorIf.getFile().getBaseName().matches("Copter.cpp")
//     // Check if the If condition compares with string "ENABLED"

//     // NOTE: Alternatively, we can check for exactly == ENABLED
//     and preprocessorIf.getIf().toString().matches("%ENABLED%")
//     // Find if there's a macro in the if condition
//     // and preprocessorIf.getIf().findRootCause() instanceof Macro

// select preprocessorIf, preprocessorIf.getIf().findRootCause().getLocation(),  "Macro call"
import cpp

from PreprocessorBranch branch, Macro macro
where 
  branch instanceof PreprocessorIf and
  branch.getFile().getBaseName().matches("Copter.cpp") and
  branch.getIf().toString().matches("%ENABLED%") and
  macro.getAnInvocation().getLocation().getStartLine() = branch.getLocation().getStartLine()
select branch, macro, "Macro '" + macro.getName() + "' is used in preprocessor if condition on line " + 
  branch.getLocation().getStartLine().toString()


// Macros from a file only
//  from Macro m
// where m.getFile().getAbsolutePath().toString().matches("Copter.cpp")
// Select only macros that start from HAL
// select m, m.getLocation().getStartLine(), m.getLocation().getStartColumn(), m.getName(), m.getFile()