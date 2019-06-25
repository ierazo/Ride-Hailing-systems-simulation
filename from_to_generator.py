import pandas as pd
import pickle
from random import choices
import numpy as np
from collections import Counter
import random
from time import time

#### Functions start here
def custom_distribution(dictionary, k=1):
    '''
    This function receives a dictionary with information 
    from - to and returns a specific destination based on 
    a custom distribution

    Parameters:
    -----------
    dictionary: a dictionary on the form {'9234':100, '92543':70, ...}
    k: the number of samples 

    Returns:
    A string, e.g. '9234'
    '''
    total = sum(dictionary.values()) # sum of total values in the dictionary 100+70+...
    population = list(dictionary.keys()) # keys '9234', '92543', ...
    weights = np.array(list(dictionary.values()))/total # weights that sum up to 1
    return choices(population, weights, k=k) # Choices function from random package

def point_sample(dictionary, zip2osmid, status='busy', move=False, destinations=None, k=1):
    '''
    busy is used to get origin-destinations for passengers
    idle is used to get origin-destination for drivers while doing roaming events
    '''
    if status == 'busy':
        if not move:
            d = dictionary['pickups']
            sample = custom_distribution(d,k=k)
            taz_point = [custom_distribution(dictionary['granular_picks'][e], k=1).pop() for e in sample]
        if move and destinations:
            d = destinations
            sample = custom_distribution(d,k=k)
            taz_point = [custom_distribution(dictionary['granular_drops'][e], k=1).pop() for e in sample]
        return [zip2osmid[v] for v in taz_point], sample
    else:
        assert (status == 'idle'), 'sense argument must be in [picks, drops]'
        if move and destinations:
            d = destinations 
        sample = custom_distribution(d, k=k)
        taz_point = [custom_distribution(dictionary['granular_picks'][e], k=1).pop() for e in sample]
        return [zip2osmid[v] for v in taz_point], sample


def my_from_to_function(i, day_hours_intervals, days_intervals, hourly_blocks, threeZip2osmid, 
            fi_zip_dict, move_dict, start5zip=None, status='busy', verbose=False):
    '''
    This function return a tuple containing the from-to street nodes values according the 
    day and time of the day.
    '''
    h_i = day_hours_intervals
    d_i = days_intervals
    def helper(i, lst):
        for idx, data in enumerate(lst[:]):
            if data[0][0] <= np.ceil(i) <= data[0][1]:
                if verbose:
                    print('returning ', data[1])
                return data[1]
            else:
                if verbose:
                    print('eliminando ',data[1])
                _ = lst.pop(0) 
    if status=='busy':
        day = helper(i, d_i)
        hour = helper(i, h_i[day])
        block = [u for u, v in hourly_blocks.items() if hour in v][0] 
        key_5zip = status+'_'+day+'_'+hour
        pickup = fi_zip_dict[key_5zip]
        pick_osmid, sample = point_sample(pickup, threeZip2osmid)
        key_move = status+'_'+day+'_'+block
        dest_move = move_dict[key_move][sample[0]]
        drop_osmid, sample_ = point_sample(pickup, threeZip2osmid, move=True, 
                    destinations=dest_move)
        return (pick_osmid, drop_osmid), (sample[0], sample_[0])
    else:
        assert(status == 'idle'), 'Status should be either busy or idle'
        day = helper(i, d_i)
        hour = helper(i, h_i[day])
        block = [u for u, v in hourly_blocks.items() if hour in v][0] 
        key_5zip = 'busy'+'_'+day+'_'+hour
        idle_start = fi_zip_dict[key_5zip]
        key_move = status+'_'+day+'_'+block
        dest_move = move_dict[key_move][start5zip]
        drop_osmid, sample_ = point_sample(idle_start, threeZip2osmid, move=True, 
                    status=status, destinations=dest_move)
        return (start5zip, drop_osmid), sample_[0]

    '''
    This is usefull if you want to follow the cab-drivers' strategy of what to do
    when the move empty. You need to include a specific start5zip id and status='idle'
    a,b = my_from_to_function(time,day_hours_intervals,days_intervals,hourly_blocks,
                threeZip2osmid,summary_dictionary_all,final_dict, verbose=True)
    '''