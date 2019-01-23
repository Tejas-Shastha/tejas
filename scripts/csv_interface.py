#!/usr/bin/env python
import numpy
import csv


def writePolicyArrayToCsv(path, policy):
    with open(path, mode='w') as policy_file:
        writer = csv.writer(policy_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(policy)

def readPolicyFromCsv(path):
    arrays = []
    with open(path) as policy_file:
        reader = csv.reader(policy_file, delimiter=',')
        for row in reader:
            arrays.append(row)
    
    intarrays = []
    for row in arrays:
        for val in row:
            intarrays.append(int(val))
    return intarrays
        
def writeQTableToCsv(path, qtable):
    with open(path, mode='w') as q_file:
        writer = csv.writer(q_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in qtable:
            writer.writerow(row)

def readQTableFromCsv(path, shape):
    qtable = numpy.empty(shape)
    with open(path) as q_file:
        reader = csv.reader(q_file, delimiter=',')
        for i,row in enumerate(reader):
            qtable[i]=row
    return qtable


def test():
    array = [1, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 1]
    qtable = numpy.random.rand(15,3)

    writePolicyArrayToCsv("test.csv", array)
    arrays = readPolicyFromCsv("test.csv")
    if array==arrays: print("Policy csv Same")
    print(arrays)

    writeQTableToCsv("qtable.csv",qtable)
    acquired = readQTableFromCsv("qtable.csv",[15,3])
    if (qtable==acquired).all: print("Qtables csv same")
    print(acquired)

if __name__=="__main__":
    test()

