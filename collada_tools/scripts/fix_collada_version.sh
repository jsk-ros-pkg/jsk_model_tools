#!/usr/bin/env bash

if [ -e $1 ]; then
    sed -i.orig -e 's@<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">@<COLLADA xmlns=\"http://www.collada.org/2008/03/COLLADASchema\" version=\"1.5.0\">@' $1;
else
    echo ";;file not found $1";
fi
