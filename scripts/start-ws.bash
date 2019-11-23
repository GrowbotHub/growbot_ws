#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

. ${DIR}/../devel_isolated/setup.bash
. ${DIR}/../devel/setup.bash
roslaunch growbothub_tlc growbothub.launch

