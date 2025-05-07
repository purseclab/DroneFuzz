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
  - DONE Get the default threshold with N runs
    - Need to actually get the rcou values to be sent to the simulation object
    - Cause TCP conn is closed after each run
  - DONE Get the DTW working
  - TODO Remove clean duplication
  - TODO Log inputs to a file
  - DONE Fix the time within the fuzzer usec (for the messages)
  - TODO Add fancy tqdm to show the progress and if bug found
  - TODO Add SIM/CFG parameters from PGFUZZ
    - Since we keep track of the time since the simulation start we can effectively set the time for the message
  - TODO Figure out to check all configurations before starting stuff
- Compare with DTW
  - Will have to save the RCOU values as we don't have MAVProxy with us anymore
