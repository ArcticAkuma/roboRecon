import importlib.util
import logging
import os
import shutil
import sys

PROJECT_PATH = os.path.dirname(os.path.realpath(__file__))
ALTERED_PATH = os.path.join(PROJECT_PATH, 'altered_files/donkeycar')

logger = logging.getLogger(__name__)

# 
def in_venv():
    return sys.prefix != sys.base_prefix

def pkg_exists(name):
    return name in sys.modules or importlib.util.find_spec(name)

def override_donkey():
    if not in_venv():
        logger.warning("Not running inside of an virtual environment!")
        val = input("Continue anyway? [y/n]")
        if val != "y" and val != "yes":
            return

    if not pkg_exists("donkeycar"): # todo: change to donkey
        logger.error("Donkeycar package not found in current environment!")
        return

    import donkeycar
    donkey_path = os.path.dirname(os.path.realpath(donkeycar.__file__))

    if (pkg_exists("pyfiglet")):
        from pyfiglet import Figlet
        f = Figlet(font='big')
        print(f.renderText('OVERRIDE WARNING'))
    else:
        print('\nOVERRIDE WARNING\n')
    print(f"Overriding files at: {donkey_path!r}")
    print('Please make sure the path above is correct and you have a backup of all files at this location.')
    print('Proceed on your own risk.\n')
    print('THIS STEP CAN NOT BE REVERTED!')
    val = input("Continue? [y/n]")
    if val != "y" and val != "yes":
        return

    counter = 0
    for path, sub_dirs, files in os.walk(ALTERED_PATH):
        for name in files:
            from_path = os.path.join(path, name)
            to_path = os.path.join(path, name).replace(ALTERED_PATH, donkey_path)

            if not os.path.exists(to_path):
                logger.warning("Missing directory: ", to_path)
                val = input("Continue anyway? [y/n]")
                if val != "y" and val != "yes":
                    continue
                os.makedirs(to_path)

            print(f"Coping {from_path} to {to_path}")
            shutil.copyfile(from_path, to_path)
            counter+=1

    print("\nOVERRIDE COMPLETE")
    print(f"This program altered {counter!r} file(s).")

if __name__ == "__main__":
    override_donkey()
