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


## TODO

- TODO Remove clean duplication ???
- TODO Figure out a way to duplicate results, how can we re-run the generate messages?
  - IN progress -> Can do that with the generated log message?
- TODO Send the fuzzing message in the same way as periodic messages
  - Can use the same function as the periodic messages
- TODO Fix to a generic mission that does everything
  - Can use a variant that forcibly changes modes
- TODO Add SIM/CFG parameters from PGFUZZ
  - Since we keep track of the time since the simulation start we can effectively set the time for the message
- TODO Figure out to check all configurations before starting stuff
- TODO Check if we can read all the files before doing stuff
- Compare with DTW
  - Will have to save the RCOU values as we don't have MAVProxy with us anymore
