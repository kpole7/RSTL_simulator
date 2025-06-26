#!/bin/bash
for i in {1..16}; do
    socat pty,raw,echo=0,link=/tmp/server$i pty,raw,echo=0,link=/tmp/client$i &
    sleep 0.1
    echo "Utworzono parę: /tmp/server$i <-> /tmp/client$i"
done

