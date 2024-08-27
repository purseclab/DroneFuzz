/**
 * @id cpp/sensor_parm_finder
 * @name Sensor Param Finder
 * @description Finds Sensor Functions 
 * @tags macro_custom 
 */

import cpp

// Find all the if directives that enable the FAST_TASK or SCHED_TASK macros
// from PreprocessorIf pbif
// select pbif
from PreprocessorIf ppIf, MacroInvocation mi
where
    ppIf.getFile().getBaseName().matches("Copter.cpp") and
    // Ensure the macro invocation is in the same file as the `#if`
    mi.getLocation().getFile() = ppIf.getLocation().getFile() and
    
    // Ensure the macro invocation is after the `#if` directive
    // mi.getActualLocation() > ppIf.getLocation() and
    
    // Ensure the macro invocation is before any `#else`, `#elif`, or `#endif` that corresponds to the `#if`
    // (
    //     not exists(PreprocessorBranchDirective ppd |
    //         ppd.getLocation() > ppIf.getLocation() and 
    //         ppd.getLocation() < mi.getLocation() and
    //         ppd = ppIf.getEndIf()
    //     )
    // ) and
    
    // Optionally, filter the macro invocation to match a specific macro (like `SCHED_TASK`)
    mi.getMacro().hasName("SCHED_TASK")

select ppIf, mi, "Macro call within #if directive"

// where ifd.getCondition().toString().matches(".*FAST_TASK.*") or ifd.getCondition().toString().matches(".*SCHED_TASK.*")
// Macros from a file only
//  from Macro m
// where m.getFile().getAbsolutePath().toString().matches("Copter.cpp")
// Select only macros that start from HAL
// select m, m.getLocation().getStartLine(), m.getLocation().getStartColumn(), m.getName(), m.getFile()