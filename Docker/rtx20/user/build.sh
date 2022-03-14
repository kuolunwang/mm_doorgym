#!/bin/sh
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_ed25519)" -f dockerfile -t argnctu/pokingbot-rl:rtx20_user .
