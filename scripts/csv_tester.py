#!/usr/bin/env python

import csv

array = [1, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 1]

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
        



writePolicyArrayToCsv("test.csv", array)
arrays = readPolicyFromCsv("test.csv")
if array==arrays: print("Same")
print(arrays)
