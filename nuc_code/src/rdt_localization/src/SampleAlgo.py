import statistics
from scipy.stats import t
import math
'''SampleAlgo is an algo that constructs a normal distribution of normal events
and use that distribution to measure when wacky shit is happening
SampleAlgo DOES NOT TREAT EACH LIDAR INDEPENDENTLY. It's implemented this way because
calibration can be done faster if it's taking in 4 lidar data at a time instead of only 1.
However, you can construct a SampleAlgo for each lidar and feed the lidar data to the corresponding SampleAlgo. Just change self.lidarsum to [0] instead of [0,0,0,0] 
and it should change correspondingly.
'''
class SampleAlgo:
    def __init__(self,tolerance_interval, sample_size = 30, calibration_size = 6):
        self.sample_size = sample_size #size of each group of data. 30 is recommended
        self.calibration_size = calibration_size
        self.sample_sum = 0
        self.data_count = 0
        self.samples = [] #sample average is stored here to calculate deviation. 
        #If there's a better method to calculate deviation without storing them, feel free to implement it
        self.lidarsum = [0,0,0,0] #sum of data from 4 lidars are stored here. 
        #When data count reach sample size, sum is divided to take average & compare to fixed interval
        self.sample_mean = 0
        self.sample_deviation = 0
        self.prediction_interval = 0
        self.tolerance = tolerance_interval 
        self.calibration_ended = False
    #take in a list and either read calibration or read with sample mean
    def read(self,val):
        if not self.calibration_ended:
            for i in val:
                self.calibration_read(i)
        else:
            return self.sample_read(val)
    '''calibration by reading all 4 lidars separately.
    Since each lidar is assumed to be independent & have the same distribution, reading 4 is faster than reading 1
    when calibration is finished, sample average & deviation is calculated to construct a fixed interval
    '''
    def calibration_read(self,val):
        if not self.calibration_ended:
            self.sample_sum+=val
            self.data_count+=1
            if self.data_count%self.sample_size == 0:
                self.samples.append(self.sample_sum/self.sample_size)
                self.sample_sum = 0
                self.curr_sample_size = 0
            if self.data_count == self.sample_size*self.calibration_size: #900 is pretty high. Change this to 180 if you're impatient
                self.sample_mean = statistics.mean(self.samples)
                self.sample_deviation = statistics.stdev(self.samples)
                self.prediction_interval = self.sample_deviation * -t.ppf((1-self.tolerance)/2,self.sample_size-1)*math.sqrt(1+(1/self.sample_size)) #formula for prediction interval
                self.samples.clear()
                self.data_count = 0
                self.calibration_ended = True
    '''read from a list of 4 integers
    when 30 samples from all 4 lidars are read, the average is taken & compare to the fixed interval from calibration
    '''
    def sample_read(self,val):
        for i in range(len(self.lidarsum)):
            self.lidarsum[i]+=val[i]
        self.data_count+=1
        if self.data_count>=self.sample_size:
            obstacle_bool = []
            for i in range(len(self.lidarsum)):
                if self.sample_mean-self.prediction_interval<self.lidarsum[i]/self.data_count<self.sample_mean+self.prediction_interval:
                    obstacle_bool.append(False)
                else:
                    obstacle_bool.append(True)
            self.lidarsum = [0,0,0,0]
            self.data_count = 0
            return obstacle_bool