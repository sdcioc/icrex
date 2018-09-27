#!/usr/bin/env bash

# Assumes http-server has been installed globally with npm
#http-server `rospack find experiment_package`/client -c-1 -p 8181
# First, we need to copy the app in the home folder
cp -r `rospack find experiment_package`/client ~/.pal/www/webapps/client/experiment

# Then, assumes pal_chrome is running, and we can send it an URL (will be latched anyway)
rostopic pub /web/go_to pal_web_msgs/WebGoTo 3 "/static/webapps/client/experiment/index.html"