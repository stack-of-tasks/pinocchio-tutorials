#!/bin/bash

NAME=eur0c.laas.fr:4567/stack-of-tasks/pinocchio-tutorials

for stage in build deploy
do (
    docker build -t $NAME/$stage -f Dockerfile.$stage .
    docker push $NAME/$stage
    ) &
done

wait
