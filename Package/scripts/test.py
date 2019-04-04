#!/usr/bin/env python
import sys, time, os
import numpy as np
import csv

def main(args):  
    
    csv_file  = open('Experiment_Data.csv', 'a')
    csv_file.write(str(56) + "," + str(0.3) + "\n")

if __name__ == '__main__':
    main(sys.argv)
