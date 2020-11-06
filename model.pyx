# cython: language_level=3

import numpy as np
from sklearn.ensemble import RandomForestClassifier
import joblib

class PyClass(object):
    def __init__(self):
        self.model = joblib.load('./random_forest.joblib')
        print("init")

    def predict_one(self, plane_invalid_page_percent, plane_valid_page_percent, plane_free_page_percent, plane_free_block_percent, has_gc_transaction, proportional_slowdown_before, fairness_before):
        x = []
        x.append(plane_invalid_page_percent)
        x.append(plane_valid_page_percent)
        x.append(plane_free_page_percent)
        x.append(plane_free_block_percent)
        x.append(has_gc_transaction)
        x.append(proportional_slowdown_before)
        x.append(fairness_before)

        # numpy type
        x = np.array([x], dtype='f')

        # predict
        ans = self.model.predict(x)
        return int(ans[0])

cdef public object createPyClass():
    return PyClass()

cdef public int predict_one_C( object p, float plane_invalid_page_percent, float plane_valid_page_percent, float plane_free_page_percent, float plane_free_block_percent, int has_gc_transaction, float proportional_slowdown_before, float fairness_before):
    return p.predict_one(plane_invalid_page_percent, plane_valid_page_percent, plane_free_page_percent, plane_free_block_percent, has_gc_transaction, proportional_slowdown_before, fairness_before)

cdef public void testing():
    print("testing")
