#!/usr/bin/env bash

# Assumes http-server has been installed globally with npm
http-server `rospack find experiment_package`/client -c-1 -p 8181
