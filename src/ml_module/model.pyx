# cython: language_level = 3
# distutils: language = c++

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
import joblib
import math
import collections

from libcpp.vector cimport vector

class RFC(object):
    def __init__(self, path='./src/ml_module/random_forest.joblib'):
        self.rfc = joblib.load(path)
        print("rfc initialized with path: " + path)
    def predict(self, x_test):
        return int(self.model.predict([x_test])[0])

class KNC(object):
    def __init__(self, n_neighbors=5, n_records=10, min_records_per_label=3):
        self.knc = KNeighborsClassifier(n_neighbors=n_neighbors)
        self.x = [collections.deque(), collections.deque()]
        self.time = [collections.deque(), collections.deque()]
        self.k = n_neighbors
        self.n_records = max(n_records, self.k)
        self.clock = self.n_records
        self.min_records_per_label = min(max(math.ceil(self.k / 2), min_records_per_label), math.ceil(self.n_records / 2))
        print("==========")
        print("knc initialized with k=%d, n_records=%d, min_records_per_label=%d" %(self.k, self.n_records, self.min_records_per_label))
        print("==========")

    def update_training_set(self, record, label):
        self.x[label].append(record.copy())
        self.time[label].append(self.next_clock())
        if len(self.x[label]) + len(self.x[1 - label]) < self.n_records:
            self.x[1 - label].append(record.copy())
            self.x[1 - label].append(self.next_clock())
        elif len(self.x[label]) + len(self.x[1 - label]) > self.n_records:
            del_label = label
            if len(self.x[0]) > self.min_records_per_label and len(self.x[1]) > self.min_records_per_label:
                del_label = 0 if self.time[0][0] < self.time[1][0] else 1
            self.x[del_label].popleft()
            self.time[del_label].popleft()

    def predict(self, x_test):
        if len(self.x[0]) + len(self.x[1]) < self.n_records:
            return int(1)
        y = [0] * len(self.x[0]) + [1] * len(self.x[1])
        x = self.x[0] + self.x[1]
        return int(self.knc.fit(x, y).predict([x_test])[0])

    def next_clock(self):
        self.clock = (self.clock + 1) % (self.n_records + 1)
        return self.clock


cdef public object rfc_init(char* path):
    return RFC(path=path)

cdef public int rfc_predict(object rfc, float pip, float pvp, float pfp, float pfb, float bip, int gt, float psd, float f):
    x = [pip, pvp, pfp, pfb, bip, gt, psd, f]
    return rfc.predict(x)


cdef public object knc_init(int n_neighbors, int n_records, int min_records_per_label):
    return KNC(n_neighbors=n_neighbors, n_records=n_records, min_records_per_label=min_records_per_label)

cdef public void knc_update_training_set(object knc, vector[float]& record, int label):
    knc.update_training_set(record, label)

cdef public int knc_predict(object knc, float bip, int gt, float psd, float f):
    x = [bip, gt, psd, f]
    return knc.predict(x)

cdef public void testing():
    print("testing")

cdef public void delete_instance(object classifier):
    del classifier
    print('delete gc_classifier success')
