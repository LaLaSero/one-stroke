#!/bin/bash

if [ $# -lt 1 ]; then
    echo "howto use: $0 <input_file.csv>"
    exit 1
fi

INPUT_FILE="$1"

X_OUTPUT_FILE="x_$(basename "$INPUT_FILE" .csv).txt"
Y_OUTPUT_FILE="y_$(basename "$INPUT_FILE" .csv).txt"

if [ ! -f "$INPUT_FILE" ]; then
    echo "error: Input File '$INPUT_FILE' not found."
    exit 1
fi

cut -d',' -f1 "$INPUT_FILE" | paste -sd, - > "$X_OUTPUT_FILE"
echo "Saved X coordinates to '$X_OUTPUT_FILE'."

cut -d',' -f2 "$INPUT_FILE" | paste -sd, - > "$Y_OUTPUT_FILE"
echo "Saved Y coordinates to '$Y_OUTPUT_FILE'."

echo "completed successfully."