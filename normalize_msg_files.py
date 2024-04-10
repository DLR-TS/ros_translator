import os
import sys
import glob
import re

def convert_camel_to_snake_case(string):
    # Check if the string is already in snake_case format or all uppercase
    if re.match(r'^[a-z]+(_[a-z]+)*$', string) or string.isupper():
        return string.lower()
    # Use regex to properly handle consecutive underscores
    return re.sub(r'(?<!^)(?=[A-Z])|(?<=\w)(?=[A-Z][a-z])', '_', string).lower()

def convert_msg_files(msg_directory):
    msg_files = glob.glob(os.path.join(msg_directory, '*.msg'))

    for msg_file in msg_files:
        print("Converting file:", msg_file)
        with open(msg_file, 'r') as file:
            lines = file.readlines()

        converted_lines = []
        for line in lines:
            # Skip comments and empty lines
            if not line.strip() or line.strip().startswith('#'):
                converted_lines.append(line)
                continue
            
            # Split each line by whitespace
            parts = line.split(' ', maxsplit=1)
            if len(parts) != 2:
                converted_lines.append(line)
                continue
            
            # Convert the variable name (second field) to snake_case and lowercase
            converted_variable_name = convert_camel_to_snake_case(parts[1].strip())
            
            # Preserve the original data type (first field) and only modify the variable name
            converted_line = parts[0] + ' ' + converted_variable_name + '\n'
            
            # Append the converted line to the list of converted lines
            converted_lines.append(converted_line)

        with open(msg_file, 'w') as file:
            file.writelines(converted_lines)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python convert_msg_files.py <msg_directory>")
        sys.exit(1)

    msg_directory = sys.argv[1]
    if not os.path.isdir(msg_directory):
        print("Error: The specified directory does not exist.")
        sys.exit(1)

    convert_msg_files(msg_directory)

