"""
@file sensor_class.py
@brief Class for reflectance sensor array.
@details
Reflectance sensor array object allows users to calibrate data using zero and normalize
and calculate the centroid of a line using readlevel and centroid.
@author Jake Rowen
@author Quinn Patterson
@date 2026-03-18
"""
from pyb import Pin
from pyb import ADC
from array import array
## @brief Reflectance sensor array class
class sensor_array:
    ## @brief Constructor for reflectance sensor array object
    ## @param pin1 Sensor Analog Pin 1
    ## @param pin2 Sensor Analog Pin 2
    ## @param pin3 Sensor Analog Pin 3
    ## @param pin4 Sensor Analog Pin 4
    ## @param pin5 Sensor Analog Pin 5
    ## @param CTRLpin Control pin for sensor array
    def __init__(self,pin1, pin2, pin3, pin4, pin5, CTRLpin):
        self._pin1 = Pin(pin1,Pin.IN) # intitialize pins as input from sensor
        self._pin2 = Pin(pin2,Pin.IN)
        self._pin3 = Pin(pin3,Pin.IN)
        self._pin4 = Pin(pin4,Pin.IN)
        self._pin5 = Pin(pin5,Pin.IN)
        self._adc1 = ADC(self._pin1) # Pins are ADC readings
        self._adc2 = ADC(self._pin2)
        self._adc3 = ADC(self._pin3)
        self._adc4 = ADC(self._pin4)
        self._adc5 = ADC(self._pin5)
        self.adc_pins = [self._adc1,self._adc2,self._adc3,self._adc4,self._adc5]
        ## @brief Array containing calibrated white color reflectance readings
        self.baseline = array('f', [0,0,0,0,0])
        self.readings = array('f', [0,0,0,0,0])
        ## @brief Array containing calibrated black color reflectance readings
        self.scalar = array('f', [1,1,1,1,1])
        ## @brief Array containing linear positions of each sensor
        self.x = array('i', [-2,-1,0,1,2])
        ## @brief Reading strength threshold for centroid calculation
        self.threshold = 0.1
        self._ctrl = Pin(CTRLpin, Pin.OUT)
        self._ctrl.high()
    ## @brief Method to read the reflectance value of each sensor
    ## @return array containing sensor readings
    def readlevel(self):
        for i in range(5):
            self.readings[i] = max(0,((self.adc_pins[i].read() / 4095 * 3.3) - self.baseline[i]) / self.scalar[i])
        return self.readings
    ## @brief Method to measure and calibrate the white color
    ## @return White color reading array
    def zero(self): #sets white color to 0 in readlevel
        for i in range(5):
            self.baseline[i] = (self.adc_pins[i].read() / 4095 * 3.3)
        return self.baseline
    
    ## @brief Method to measure and calibrate black color
    ## @return Black color array readings
    def normalize(self): #sets black to 1 in readlevel
        for i in range(5):
            self.scalar[i] = (self.adc_pins[i].read() / 4095 * 3.3)-self.baseline[i]
        return self.scalar

    ## @brief Method to calculate the centroid of a line
    ## @return Location of centroid of line with respect to sensors. None if reading is weak.
    def centroid(self): #Centroid
        readings = self.readlevel()
        numerator = 0
        for i in range(5):
            numerator+=(self.x[i]*readings[i])
        wstrength = sum(readings)
        if wstrength<self.threshold:
            return None
        center = numerator/wstrength
        return center
        
        
        
