# rosplan_gui
This application acts as an GUI to the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) Knowledge Base. 

Currently, users can view KB data as it updates. Users may also update and remove the most recent predicate data through the application. The user cannot currently add new instances or proposition data: this is in development.

## Installation

### Pre-requisites
Reccommended setup: Ubuntu 18.04 and ros-melodic \
Required software: [ROSPlan](https://github.com/KCL-Planning/ROSPlan)

(optional) A package manager: either [Anaconda](https://www.anaconda.com/) or [virtualenv](https://pypi.org/project/virtualenv/).

### Installation Guide
Clone the repository
```
git clone https://github.com/H0PP3R/rosplan_gui
```

Navigate inside the repository directory
```
cd rosplan_gui
```

#### Install Dependencies
If using Anaconda:
```
conda env create --name <envname> --file=environments.yml
```

If using virtualenv
```
python3 -m venv <envname>
source <envname>/bin/activate
pip install -r requirements.txt
```

If using the base PC
```
pip install -r requirements.txt
```

## Running Instructions
Launch the ROSPlan launchfile
```
roslaunch <launchfile>.launch
```

Run the main file
```
python <path-to-repository>/src/main.py
```
