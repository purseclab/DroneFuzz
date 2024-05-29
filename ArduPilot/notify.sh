#!/bin/bash
log=$(tail -n 7 $PGFUZZ_HOME/ArduPilot/fuzzing.log)
curl -s -X POST -d chat_id=$CHAT_ID -d text="Fuzzer on $(hostname) exited with following: $log" "https://api.telegram.org/bot$TOKEN/sendMessage" > /dev/null
