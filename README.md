# RoboRecon - a minor-level university robotics project

![License](https://img.shields.io/badge/license-MIT-green)

### Project Overview
This repository contains the code for a minor-level university project focused on developing a self-driving car prototype.
Currently, there is only the code adjustments to the [Donkeycar](https://github.com/autorope/donkeycar) software published.

### Donkeycar Alterations
Patches regarding Donkeycar are made to enable easy implementation of ROS 1.
They are applied through a simple script overriding changed classes directly in the python environment.</p>
**Please consider to clone the above-mentioned official repository to apply changes.**<br/>
This basic solution was mainly developed to simplify grading of the project by condensing self-modified files only.

````shell
python override.py
````

### Grading
All self-created or 'significantly' altered files in [/altered_files](https://github.com/ArcticAkuma/roboRecon/tree/main/altered_files) will be marked as followed.
Please note, this annotation will be limited to this package only. Files in other packages are self-created.
This should make it easier to find relevant code by e.g. searching for this markdown in an IDE.</p>
*Legacy branches will not be marked like this. Expect all files there to be self-created 
apart from those clearly in sub-packages named accordingly to the origin software.*

#### Self-created files:
````python
    ### SELF-MAINTAINED
    # <explanation>
    
    class GradingExample:
        def __init__(self):
            print('grading_example')
        
        #<explanation>
        def grading_example:
            pass
````
#### Altered files:
````python
    ### ALTERED START ###
    # <explanation>
    
    print('grading_example')

    ### ALTERED END ###
````