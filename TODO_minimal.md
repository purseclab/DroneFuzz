# TODO

- Fuzz things
  - For now just using the another thread
  - DONE Fuzzer stats
    1. Simulations done
    2. Messages sent
    3. Average time taken
  - DONE Ensure that we run forever
  - DONE Parse the mapping
  - DONE Handle periodic messages
  - DONE Exit simulation if PreArm error
  - TODO Get the default threshold with N runs
  - TODO Get the DTW working
  - TODO Log inputs to a file
  - TODO Fix the time within the fuzzer usec (for the messages)
  - TODO Add fancy tqdm to show the progress and if bug found
  - TODO Add SIM/CFG parameters from PGFUZZ
    - Since we keep track of the time since the simulation start we can effectively set the time for the message
  - TODO Figure out to check all configurations before starting stuff
- Compare with DTW
  - Will have to save the RCOU values as we don't have MAVProxy with us anymore
