# Command Voice Recognition for TIAGo Robot Using MFCC and MLP

Assisted navigation is proposed for the PAL-Robotics TIAGo robot, ROS-Kinetic distribution, based on command voice recognition. An implementation of isolated word recognition using artificial neural networks (MLP) and voice feature extractor (MFCC).

Words Recognized:

* backward
* forward
* go
* left
* right
* stop


To avoid false voice commands and noise interference, it's proposed to use a keyword "go" to start. First, the TIAGo robot awaits the keyword. Then wait for the voice command navigation ("backward","forward","left","right"). If the voice command is correct, the mobile base is running through that command. Finally, there are two options to stop the robot, the "stop" command or by canceling the "go" keyword command, through the utterance for the second time.

### References

Python Speech Features
https://github.com/jameslyons/python_speech_features

Speech Recognition ANN Implementation
https://github.com/RomelioTavas/Speech-Recognition-ANN
