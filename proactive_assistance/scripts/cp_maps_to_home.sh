#! /bin/sh
#
# Copies the maps from a given pkg (robot) to $HOME/.pal; creates the target
# folder if it doesn't exist.

TARGET=$HOME/.pal/tiago_dual_maps/configurations

PKG=$1

if [ -z $PKG ]; then
    echo "Usage: $0 <pkg>"
    exit 1
fi

SOURCE=$(rospack find $PKG)

cp -r $SOURCE/configurations/proactive_demo_simulation $TARGET

echo "Done."