from __future__ import division 
import numpy as np 
import scipy.io.wavfile as wav

from features import mfcc

class TestingNetwork:

	weights = [];

	def __init__(self,layerSize,weights):

		self.weights = weights

                # input-layer, xi+bias
                self.yib = []
                
                # hidden-layer, yj+bias
                self.yjb = []

                # Logistic-function parameter 
                self.a = 1

                if len(layerSize) != 3:
                    raise Warning("The layer size must be 3")

	def forwardProc(self,xi):

                # xi + bias
                self.yib=np.vstack(np.concatenate([np.array([1]),xi[0]]))

                # Hidden layer, net(j): vj=wji*yi, yj=f(vj)
                vj=self.weights[0].dot(self.yib)
                yj=self.sgm(vj)

                # yj + bias
                self.yjb=np.vstack((np.array([1]),yj))

                # Output layer, net(k): vk=wkj*yj, yk=f(vk)
                vk=self.weights[1].dot(self.yjb)
                yk=self.sgm(vk)

                return yk.T

	def sgm(self,x,Derivative=False):
                
		if not Derivative:
			return 1/ (1+np.exp(-self.a*x))
		else:
			out = self.sgm(x)
			return self.a*out*(1-out)


def testInit(mlp_layers,filename_ann1):
	#Setup Neural Network
        f1 = open(filename_ann1, "rb")
	weights  = np.load(f1)
	testNet = TestingNetwork(mlp_layers,weights)
	return testNet

def extractFeature(soundfile):
	#Get MFCC Feature Array
	(rate,sig) = wav.read(soundfile)
	duration = len(sig)/rate;	
	mfcc_feat = mfcc(sig,rate,winlen=duration/20,winstep=duration/20)
	s = mfcc_feat[:20]
	st = []
	for elem in s:
		st.extend(elem)
	st /= np.max(np.abs(st),axis=0)
	inputArray = np.array([st])
	return inputArray

def feedToNetwork(words,inputArray,testNet):
	#Input MFCC Array to Network
	outputArray = testNet.forwardProc(inputArray)

	#if the maximum value in the output is less than
	#the threshold the system does not recognize the sound
	#the user spoke

	indexMax = outputArray.argmax(axis = 1)[0]
			
	# Mapping each index to their corresponding meaning

        # op1:
	#return words[indexMax]

        # op2:
        outn = outputArray[0]
        thOut = {'backward': 0.8,
                 'forward':  0.8,
                 'go':       0.9,
                 'left':     0.9,
                 'right':    0.9,
                 'stop':     0.7}
        if outn[indexMax] > thOut[words[indexMax]]:
            return words[indexMax]

if __name__ == "__main__":

        words = ['backward','forward','go','left','right','stop']

        # MLP Network with 3 layers (in, hidden, out)
        mlp_layers = (260,200,6)
        print "Testing MLP " + str(mlp_layers)

        filename_ann1 = "network/command_mlp.npy"
        filename_test = "test_files/test.wav"
        
	print("Testing: test_files/test.wav")
        testNet = testInit(mlp_layers,filename_ann1)
	inputArray = extractFeature(filename_test)
        testStr = feedToNetwork(words,inputArray,testNet)
        print(testStr)





	


	
		


