# Natural Language Understanding in Robotics

Excercises and notes for the lecture «Natural Language Understanding in Robotics»

Active Vision Group – AGAS

Universität Koblenz-Landau

©2019 Mauricio Matamoros

# Contents
This repository contains the following files and folders

- `turtle_nlp`: ROS package with the nlp examples
- `pocketsphinx`: ROS package of PocketSphinx ASR
- `Chatbot-Eliza-1.08.tar.gz`: Sources for the chatbot Eliza
- `slides.pdf`: The presentation slides
- `README.md`: This document


# Excercises
## Prerrequisites
ROS of course. The ASR package depends on Pocketsphinx Python. Install it with pip via

```bash
python -m pip install --upgrade pip setuptools wheel
pip install --upgrade pocketsphinx
```

## Installation

Installing the package with the examples follows the normal ROS conventions.
Copy the package turtle_nlp into a ROS workspace.

### Execution
Start ROSCore. To run the excercises, simply run any of the the four example nodes of the `turtle_nlp` package as follows:

### 1. Start a turtlesim node
```bash
rosrun turtlesim turtlesim_node

```
### 2. Start the example file
```bash
rosrun turtle_nlp example1.py
```

### 3. Start the ASR node (optional)
This step is only required if you want to control the turtle with voice
```bash
rosrun pocketsphinx asr.py
```

### Eliza
To execute Eliza, uncompress the sources, build, and install


```bash
tar -xzvf Chatbot-Eliza-1.08.tar.gz
cd Chatbot-Eliza-1.04
perl Makefile.PL
make
sudo make install
```
The sources come with the software in several languages. The executables are in the `examples` subfolder. Run:

**English**
```bash
cd examples
./simple
```

**German**
```bash
cd examples
./deutsch
```
