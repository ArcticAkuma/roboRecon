import importlib.util
import logging
import os
import shutil
import sys

# Localize project files on deployed system.
PROJECT_PATH = os.path.dirname(os.path.realpath(__file__))
ALTERED_PATH = os.path.join(PROJECT_PATH, '../altered_files/donkeycar')
SCRIPT_PATH = os.path.join(PROJECT_PATH, '../scripts')

logger = logging.getLogger(__name__)

# Checks if application is run from within a virtual environment.
def in_venv():
    return sys.prefix != sys.base_prefix

# Checks if package exists within a current environment.
def pkg_exists(name):
    return name in sys.modules or importlib.util.find_spec(name)

# Runs overriding process for every file in sources_path.
# All files will be copied to execution_path.
def run_override(sources_path, execution_path):
    counter = 0
    new_folders = 0
    new_files = 0
    for path, sub_dirs, files in os.walk(sources_path):
        for name in files:
            # Fetch full origin and target path.
            from_path = os.path.join(path, name)
            to_path = os.path.join(path, name).replace(sources_path, execution_path)

            # Get directory of path.
            # Check if directory exists at target location.
            # If not prompt user to confirm process and create directory.
            dir = os.path.split(to_path)[0]
            if not os.path.exists(dir):
                logger.warning(f"Folder does not exist in donkeycar: {dir}")
                val = input("Continue anyway? [y/n]")
                if val != "y" and val != "yes":
                    continue
                os.makedirs(dir)
                new_folders += 1

            # Copy file to target location.
            if not os.path.exists(to_path):
                new_files += 1
            print(f"Coping {from_path} to {to_path}")
            shutil.copyfile(from_path, to_path)
            counter += 1
    return counter, new_folders, new_files


def override_donkey():
    # Check if application runs within a virtual environment.
    # If not prompt user to confirm changes to global environment.
    if not in_venv():
        logger.warning("Not running inside of an virtual environment!")
        val = input("Continue anyway? [y/n]")
        if val != "y" and val != "yes":
            return

    # Check if donkeycar package is installed in current environment.
    if not pkg_exists("donkeycar"): # todo: change to donkey
        logger.error("Donkeycar package not found in current environment!")
        return

    # Fetch location of donkeycar package in current environment.
    import donkeycar
    donkey_path = os.path.dirname(os.path.realpath(donkeycar.__file__))

    # Prompt user with warning.
    if (pkg_exists("pyfiglet")):
        from pyfiglet import Figlet
        f = Figlet(font='big')
        print(f.renderText('OVERRIDE WARNING'))
    else:
        print('\nOVERRIDE WARNING\n')
    print(f"Overriding files at: {donkey_path}")
    print('Please make sure the path above is correct and you have a backup of all files at this location.')
    print('Proceed on your own risk.\n')
    print('THIS STEP CAN NOT BE REVERTED!')
    val = input("Continue? [y/n]")
    if val != "y" and val != "yes":
        return

    # Run override process .
    counter, new_folders, new_files = run_override(ALTERED_PATH, donkey_path)
    counter2, new_folders2, new_files2 = run_override(SCRIPT_PATH, donkey_path + "/templates")

    # Give user some data regarding process completion.
    print("\nOVERRIDE COMPLETE")
    print(f"This program altered {counter+counter2} file(s).")
    print(f"{new_folders+new_folders2} new folder(s) and {new_files+new_folders} new file(s) were created.")

if __name__ == "__main__":
    override_donkey()
