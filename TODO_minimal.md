# TODO List

## DONE

- For now just using the another thread
- DONE Fuzzer stats
    1. Simulations done
    2. Messages sent
    3. Average time taken
- DONE Ensure that we run forever
- DONE Parse the mapping
- DONE Handle periodic messages
- DONE Exit simulation if PreArm error
- DONE Get the default threshold with N runs
  - Need to actually get the rcou values to be sent to the simulation object
  - Cause TCP conn is closed after each run
- DONE Get the DTW working
- DONE Patch the random messages to generate the correct fields
- DONE Track the DTW detection time and save to a file
- DONE Log inputs (buggy) to a file
- DONE Fix the time within the fuzzer usec (for the messages)
- DONE Fix the oracle DTW
- DONE Add fancy tqdm to show the progress and if bug found
- DONE Fix the base distance with random messages
  - 2025-05-08 09:24 Currently running a mission with patched bug to check
- DONE Figure out if we can use the ENUMs somehow actually
- DONE Save to local file
- DONE Figure out how to effectively fuzz in Auto mode
- DONE Figure out param issue
- DONE Fix the fuzzing inside auto mode?
- DONE Send the fuzzing message in the same way as periodic messages
  - Can use the same function as the periodic messages
- DONE Fix issues with Mount bug
- DONE Fix the mutation of the RF Bug as we find a violation in each run (the distance calculation was wrong)
- DONE HIGH Figure out why the freaking 2nd mission is not working ATM
- DONE Error out only if the command is Mission/Mode/Arm request, else can be just a warning

## TODO

- TODO Figure out a way to duplicate results, how can we re-run the generate messages?
  - IN progress -> Can do that with the generated log message?
- TODO Fix to a generic mission that does everything
  - Can use a variant that forcibly changes modes
  - But then how do we actually do that? We take of a mission and then switch modes?
    -> Need to carefully craft a mission that makes valid sense 
- TODO Figure out better naming for the fuzz interval -> It's supposed to denote the frequency or something
- TODO Figure out why the tqdm thing gets messed up everytime :|
- TODO Add SIM/CFG parameters from PGFUZZ
  - Since we keep track of the time since the simulation start we can effectively set the time for the message
- TODO Figure out to check all configurations before starting stuff
- TODO Check if we can read all the files before doing stuff


## Steps to full scale evaluation 
- TODO Finish up the mapping for all peripherals 
- TODO Run eval on current dataset of 10 bugs
- TODO Figure out the ideal testing times based on previous fuzzing papers for drones
  - 1. RVFuzzer + PGFUzz + sensorFuzz + SwarmFuzz
  - 2. Then figure out the co-relation from original evals

## Misc Notes
- Compare with DTW
  - Will have to save the RCOU values as we don't have MAVProxy with us anymore

## STACK
2. TODO Finish up the mapping for all peripherals 
1. Get results and compare?