#!/bin/sh

F=$(mktemp /tmp/plot_moveit_trajectory_XXXX)

rostopic echo -n1 /move_group/display_planned_path > "$F"

cat "$F" | sed -n 's/.*positions: \[//; T; s/, /,/g; T; s/\]$//g; T; p' > "$F.pos"
cat "$F" | sed -n 's/.*velocities: \[//; T; s/, /,/g; T; s/\]$//g; T; p' > "$F.vel"

octave-cli --eval "P=csvread('$F.pos'); V=csvread('$F.vel'); plot(P); figure; plot(V); waitforbuttonpress"

rm "$F" "$F.pos" "$F.vel"
