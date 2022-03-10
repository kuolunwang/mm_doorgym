#!/bin/sh
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f dockerfile -t argnctu/pokingbot-rl:rtx30_user .
