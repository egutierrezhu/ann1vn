# Command Voice Recognition for TIAGo Robot Using MFCC and MLP

An implementation of a voice command recognition system with reject option and sound detection in real time, for PAL Robotics' TIAGo robot via ROS. The system implemented is based on the Mel-frequency Cepstral Coefficients (MFCC) and the Multi-layer Perceptron (MLP). This ANN-based model was trained using the Speech Commands V2 dataset [1].

Recognized words:

* backward
* forward
* go
* left
* right
* stop

To avoid false positives caused by noise interference, it's proposed to use a keyword ("go") to start. First, the TIAGo robot awaits the keyword. Then wait for the voice command navigation ("backward","forward","left","right"). If the voice command is correct, the mobile base is running through that command. Finally, there are two options to stop the robot, the "stop" command or by canceling the "go" keyword command, through the utterance for the second time.

Previous versions:

Python Speech Features
https://github.com/jameslyons/python_speech_features

Speech Recognition ANN Implementation
https://github.com/RomelioTavas/Speech-Recognition-ANN

### References

[1] Warden, P. (2018). Speech commands: A dataset for limited-vocabulary speech recognition. arXiv preprint arXiv:1804.03209.
