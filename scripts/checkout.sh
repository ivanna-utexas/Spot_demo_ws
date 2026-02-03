#!/bin/bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(fullpath "$THIS_DIR/..")"
SRC_DIR="$PROJECT_ROOT/src"

cd $SRC_DIR

function clone_or_pull {
    REPO_BRANCH=$1
    REPO_URL=$2
    REPO_DIR=$3

    if [ -d "$REPO_DIR/.git" ]; then
        echo "Pulling latest changes for $REPO_DIR"
        cd "$REPO_DIR"
        git pull origin "$REPO_BRANCH"
        git submodule update --init --recursive
        cd "$SRC_DIR"
    else
        echo "Cloning $REPO_URL into $REPO_DIR"
        git clone -b "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR"
    fi
}

clone_or_pull main https://github.com/bdaiinstitute/spot_ros2.git spot_ros2
clone_or_pull master https://github.com/ut-amrl/spot_nav.git spot_nav
clone_or_pull master https://github.com/ut-amrl/ironback_description.git ironback_description
