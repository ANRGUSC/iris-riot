#!/usr/bin/env python3
import sys, os, shutil

if len(sys.argv) < 2:
    print("Please specify the test file to insert or 'remove' to remove the test code.")
    print("Note that removing a test will not move the test file back into the "
          "'tests/' directory. Please manually save your changes if you have made edits.")
    print("Examples:")
    print("python3 insert_test.py tests/hdlc_test.c")
    print("python3 insert_test.py remove")
    sys.exit()

if sys.argv[1] == 'remove':
    print("Removing test code and restoring original main.c...")
    os.remove("main.c") 
    shutil.move("tests/main_backup/main.c", "./main.c")
    os.rmdir("tests/main_backup")
    sys.exit()

try:
    os.mkdir("tests/main_backup")
    shutil.move("./main.c", "tests/main_backup/main.c")
except OSError:
    print("main.c backup already exists. skipping this step and inserting target test...")

shutil.copy(sys.argv[1], "./main.c")


