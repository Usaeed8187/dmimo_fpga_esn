#!/usr/bin/env python3
import sys
import os

def print_file_contents(file_paths):
    """
    Print the contents of files specified by their paths.
    Each file's output is preceded by its name and followed by a separator.
    """
    for i, file_path in enumerate(file_paths):
        try:
            with open(file_path, 'r') as file:
                content = file.read()

                # Print file name
                print(f"{file_path}:")
                # Print content
                print(content)

                # Print separator if not the last file
                if i < len(file_paths) - 1:
                    print("\n---\n")

        except FileNotFoundError:
            print(f"{file_path}: File not found")
        except PermissionError:
            print(f"{file_path}: Permission denied")
        except Exception as e:
            print(f"{file_path}: Error reading file - {str(e)}")

if __name__ == "__main__":
    # Check if file paths are provided as arguments
    if len(sys.argv) < 2:
        print("Usage: python print_file_contents.py file1 [file2 ...]")
        sys.exit(1)

    # Get file paths from command-line arguments
    file_paths = sys.argv[1:]

    # Print contents of specified files
    print_file_contents(file_paths)

