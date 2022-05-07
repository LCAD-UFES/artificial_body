#!/bin/bash

for file in $1/*.stl
do
  blender --background --python convert_stl_to_blend.py -- "$file" "${file%.stl}.obj"
done
