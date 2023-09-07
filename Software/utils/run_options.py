# run options

class RunOpts():
    '''
    Class for run options of an experiment. Users can specify data collection
    options and save options for data (by default, all values are True)
    '''
    def __init__(self):
        # collect Data options
        self.collectData = True # collects Ts and total Intensity
        self.collectSpatialTemp = True # collects average spatial temps and entire image capture
        self.collectEntireSpectra = True # collects entire intensity spectrum
        self.collectOscMeas = True # collects oscilloscope measurements
        self.collectEmbedded = True # collects embedded measurements

        # save data options
        self.saveData = True # saves inputs and outputs to a file timeStamp_dataCollectionOL.csv
        self.saveSpatialTemp = True # saves spatial temperature values to a file timeStamp_dataCollectionSpatialTemps.csv
        self.saveSpectra = True # saves entire spectrum at each sampling time to a file timeStamp_dataCollectionSpectra.csv
        self.saveOscMeas = True # saves oscilloscope measurements to a file timeStamp_dataCollectionOscilloscope.csv
        self.saveEmbMeas = True # saves embedded measurements to a file timeStamp_dataCollectionEmbedded.csv
        self.saveEntireImage = True # saves the entire thermal image data to a file timeStamp_dataCollection.npy

        self.tSampling = 1.0

    def setSamplingTime(self, tSampling):
        if self.collectOscMeas == True:
            if tSampling > 0.8:
                print('WARNING: sampling time may be greater than measurement collection + input actuation time!!')
        self.tSampling = tSampling
        return