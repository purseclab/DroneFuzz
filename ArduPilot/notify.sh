#!/bin/bash
curl -s -X POST -H "Content-Type:multipart/form-data" -F chat_id=$CHAT_ID -F text="The script exited" "https://api.telegram.org/bot$TOKEN/sendMessage" > /dev/null
