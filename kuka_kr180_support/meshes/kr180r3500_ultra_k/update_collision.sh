#!/bin/bash
find visual -type f \( -iname "*.dae" -o -iname "*.stl" \) -print0 | while read -rd $'\0' MESH
do
    OUTFILE=$(echo "${MESH}" | cut -d'/' -f2- | cut -d'.' -f1)
    CMD="meshlabserver -i ${MESH} -o collision/${OUTFILE}.stl -s script.mlx"
    echo "${CMD}"
    eval "${CMD}"
done