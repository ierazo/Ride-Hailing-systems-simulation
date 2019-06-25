"""
@authors: Ignacio Erazo and Rodrigo de la Fuente
"""
##### Importing modules
import numpy as np
import igraph
import time
import pickle
##### Other .py modules 
from from_to_generator import my_from_to_function
from CreatingGraph import *
from Classes import *

##### First all the data required for the simulations is loaded

##### Getting the TAZ Nodes
""" TAZList is the list with all the nodes that represent different TAZ. Some of them are represented
    by the same node, but this list does not repeat those values. """
start = time.time()
pickle_in = open("3zip2id.pickle","rb")  
TAZ = pickle.load(pickle_in)              
TAZList = []
for key,i in TAZ.items():
    if i in TAZList:
        a=0
    else:
        TAZList.append(str(i))
pickle_in.close()                         
end = time.time()
print("Total time needed to get the TAZList was: " + str(end-start))

#### Reading the 969 * 969 dictionary (TAZ * TAZ)
start = time.time()
pickle_in = open("SmallDict.pickle","rb") 
TAZDict = pickle.load(pickle_in)          
pickle_in.close()                        
end = time.time()
print("Total time needed to get the TAZDict dictionary was: " + str(end-start))

##### Here the vertex number in the igraph representation of the graph is mapped to the respective vertex name
start = time.time()
pickle_in = open("VertexNumToNode.pickle","rb")
MapVertexToNode = pickle.load(pickle_in)         
pickle_in.close()                         
end = time.time()
print("Total time needed to get the MapVertex dictionary was: " + str(end-start))

##### 11372 * 969 Dictionary, represents all nodes vs all TAZ nodes. It contains less information than the
##### 969 * 969 dictionary (that one includes the routes too)
start = time.time()
pickle_in = open("FullDict.pickle","rb") 
FullDict = pickle.load(pickle_in)        
pickle_in.close()                        
end = time.time()
print("Total time needed to get the FullDict dictionary was: " + str(end-start))

##### This dictionary maps the 3Zips to their 5Zips
pickle_in = open("Dict3Zipto5Zip.pickle","rb") 
Dict3ZipTo5Zip = pickle.load(pickle_in)          
pickle_in.close()                        

##### Here the nodes names are mapped to their respective 3Zips
pickle_in = open("DictNodeTo3zip.pickle","rb") 
DictNodeTo3Zip = pickle.load(pickle_in)         
pickle_in.close()                    

##### Loading the arrival times
start = time.time()
#pickle_in = open("Real10ReplicasSFCorr0.pickle","rb")  # The arrival times used in the study, 36% marketshare SF
pickle_in = open("SmallInstances10repVar1Cor0.pickle","rb") # Small instances, just 1 hour of simulation
ArrivalsTimes = pickle.load(pickle_in)     
pickle_in.close()                        
end = time.time()
print("Total time needed to get the ArrivalTimes of the small instances was: " + str(end-start))

##### Loading data required to add the spatial behavior to the arrival times
pickle_in = open("3zip2id.pickle","rb") 
threeZip2osmid = pickle.load(pickle_in)         
pickle_in.close()                    

pickle_in = open("demand_dictionary.pickle","rb") 
summary_dictionary_all = pickle.load(pickle_in)         
pickle_in.close()  

pickle_in = open("from_too_dict.pickle","rb") 
final_dict = pickle.load(pickle_in)         
pickle_in.close()  

##### Now some general conditions for the simulation will be established

### Conditions so the function creating spatio arrivals works
hourly_blocks = {'Morning':('6_7','7_8','8_9','9_10','10_11'),
            'Noon':('11_12','12_13','13_14','14_15'),
            'Afternoon':('15_16','16_17','17_18','18_19'),
            'Evening':('19_20','20_21','21_22','22_23','23_24'),
            'Night':('0_1','1_2','2_3','3_4','4_5','5_6')}
hours_list = [(1,'0_1'),(2,'1_2'),(3,'2_3'),(4,'3_4'),(5,'4_5'),(6,'5_6'),(7,'6_7'),(8,'7_8'),
        (9,'8_9'),(10,'9_10'),(11,'10_11'),(12,'11_12'),(13,'12_13'),(14,'13_14'),(15,'14_15'),
        (16,'15_16'),(17,'16_17'),(18,'17_18'),(19,'18_19'),(20,'19_20'),(21,'20_21'),(22,'21_22'),
        (23,'22_23'),(24,'23_24')]
days_list = [(0,'Monday'),(1,'Tuesday'),(2,'Wednesday'),(3,'Thursday'),(4,'Friday'),
        (5,'Saturday'),(6,'Sunday')]

hour_in_sec = 60**2
day_in_sec = 24*hour_in_sec

### Parameters that will be used to create drivers
"""These parameters were tuned by simulating the creation of drivers, results had to be similar
    to the ones presented in the dataset used."""
Proba = []
newarray1 = [0.002,0.002,0.002,0.007,0.011,0.03,0.055,0.028]
newarray2 = [0.03,0.03,0.03,0.025,0.025,0.03,0.04,0.05]
newarray3 = [0.085,0.095,0.04,0.05,0.04,0.03,0.015,0.012]
newarray4 = [0.07,0.105,0.07,0.07,0.055,0.055,0.05,0.018]
# From Monday to Thursday
for i in range(0,4):
    Proba.extend(newarray1)
    Proba.extend(newarray2)
    Proba.extend(newarray3)
# Friday
Proba.extend(newarray1)
Proba.extend(newarray2)
Proba.extend(newarray4)
# Saturday and Sunday
for i in range(0,2):
    Proba.extend(newarray1)
    Proba.extend(newarray2)
    Proba.extend(newarray3)
for i in range(0,len(Proba)):
    Proba[i]=Proba[i]/1.6

##### Establishing the scenarios to test and their particular queues 
Cars = [CarsNoRoaming,CarsSingleRandomRoaming,CarsNearestHotspot,CarsSpecificHotspot]
Queue = [ListQueueNoRoaming,ListQueueSingleRandomRoaming, ListQueueNearestHotspot, ListQueueSpecificHotspot]


################ Simulation starts here

for REPLICAS in range(1,10):
    ##### Lists are created to receive the basic simulation results
    PassengerNumber = [ [] , [] , [] , []  ] 
    AcceptedDrives = [ [] , [] , [] , [] ]  
    NonDriverAvailable = [ [] , [] , [] , []  ]
    DriverFarAway = [ [] , [] , [] , []  ]
    CarsNumber= [ [] , [] , [] , [] ]
    DriverRejecting = [ [] , [] , [] , [] ]
    DistWithPassengers = [ [] , [] , [] , []  ]
    DistanceIdle = [ [] , [] , [] , []  ]
    DistToPas = [ [] , [] , [] , []  ]
    NumRoamings = [ [] , [] , [] , []  ]

    ##### Time starts ticking
    start = time.time()
    ##### A new random seed is established for every replica, so arrivals have different origin-destination
    np.random.seed(REPLICAS) 

    ##### Passenger's arrivals are being loaded
    Arriv=ArrivalsTimes[REPLICAS]*3600 ### Arrival times are in hours, so they are adjusted

    ##### This step is required to get the origin-destination pair for each passenger depending on his arrival time
    hours_intervals = [(((i-1)*hour_in_sec,i*hour_in_sec), tag) if i==1 else
                (((i-1)*hour_in_sec+1,i*hour_in_sec), tag) for i, tag in hours_list]
    days_intervals = [((i*day_in_sec,(i+1)*day_in_sec), tag) if i==0 else 
                ((i*day_in_sec+1,(i+1)*day_in_sec), tag) for i, tag in days_list]
    day_hours_intervals = {j:[((k[0][0]+i[0],k[0][1]+i[0]),k[1]) for k in hours_intervals] for i, j in days_intervals}

    id2id = [None]*len(Arriv[0]) # it is easier to index than to append
    five2five = [None]*len(Arriv[0])

    ##### This function provides the origin-destination pair, using real data and the arrival time
    for i, tm in enumerate(list(Arriv[0])):
        a1, b1 = my_from_to_function(tm, day_hours_intervals, days_intervals, hourly_blocks,
                threeZip2osmid, summary_dictionary_all, final_dict, verbose=False)
        id2id[i] = a1
        five2five[i] = b1

    ##### Here we are using the arrival locations to get the hotspots
    ArrivalsList = [None]* len(id2id)
    for i in range(0,len(id2id)):
        ArrivalsList[i]=id2id[i][0][0]
    
    ArrivalsInHotspots,Hotspots = DiscoveringHotspots(TAZList,ArrivalsList)

    ##### The hotspot information is provided to the scenarios that need them
    CarsSpecificHotspot.Hotspots = Hotspots
    CarsSpecificHotspot.PrioritiesHotspots = ArrivalsInHotspots
    CarsSpecificHotspot.RoutedCarsToHotspots = []
    for i in range(0,len(Hotspots)):
        CarsSpecificHotspot.RoutedCarsToHotspots.append(0)
    CarsNearestHotspot.Hotspots = Hotspots

    ##### Now the drivers arrivals are being created
    NumCars = 16200   ##### this is 36% of the 45000 drivers in San Francisco
    BlockOfHours = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,
    24*1+1,24*1+2,24*1+3,24*1+4,24*1+5,24*1+6,24*1+7,24*1+8,24*1+9,24*1+10,24*1+11,24*1+12,24*1+13,24*1+14,24*1+15,24*1+16,24*1+17,24*1+18,24*1+19,24*1+20,24*1+21,24*1+22,24*1+23,24*1+24,
    24*2+1,24*2+2,24*2+3,24*2+4,24*2+5,24*2+6,24*2+7,24*2+8,24*2+9,24*2+10,24*2+11,24*2+12,24*2+13,24*2+14,24*2+15,24*2+16,24*2+17,24*2+18,24*2+19,24*2+20,24*2+21,24*2+22,24*2+23,24*2+24,
    24*3+1,24*3+2,24*3+3,24*3+4,24*3+5,24*3+6,24*3+7,24*3+8,24*3+9,24*3+10,24*3+11,24*3+12,24*3+13,24*3+14,24*3+15,24*3+16,24*3+17,24*3+18,24*3+19,24*3+20,24*3+21,24*3+22,24*3+23,24*3+24,
    24*4+1,24*4+2,24*4+3,24*4+4,24*4+5,24*4+6,24*4+7,24*4+8,24*4+9,24*4+10,24*4+11,24*4+12,24*4+13,24*4+14,24*4+15,24*4+16,24*4+17,24*4+18,24*4+19,24*4+20,24*4+21,24*4+22,24*4+23,24*4+24,
    24*5+1,24*5+2,24*5+3,24*5+4,24*5+5,24*5+6,24*5+7,24*5+8,24*5+9,24*5+10,24*5+11,24*5+12,24*5+13,24*5+14,24*5+15,24*5+16,24*5+17,24*5+18,24*5+19,24*5+20,24*5+21,24*5+22,24*5+23,24*5+24,
    24*6+1,24*6+2,24*6+3,24*6+4,24*6+5,24*6+6,24*6+7,24*6+8,24*6+9,24*6+10,24*6+11,24*6+12,24*6+13,24*6+14,24*6+15,24*6+16,24*6+17,24*6+18,24*6+19,24*6+20,24*6+21,24*6+22,24*6+23,24*6+24]
    ProbStartingAtBlock = Proba #### This parameter was tuned before

    for i in range(0,len(BlockOfHours)):
        BlockOfHours[i]=3600*BlockOfHours[i]   ##### Here the blocks are adjusted to be in seconds
    
    ##### We get the weekly schedule (Arrival and Tentative Leaving times for each driver)
    DriversArrivalEvents,DriversLeavingEvents = GenerateCarSchedules(NumCars,
    BlocksOfHours=BlockOfHours,ProbStartingAtBlock=ProbStartingAtBlock
    , ShiftFunction=np.random.triangular , Parameters=[1*3600,3*3600,6*3600])
    end = time.time()
    print("The setup of replica " + str(REPLICAS) + " took " + str(end-start) )
    

    ##### Creating the simulation scenarios
    for j in range(0,len(Cars) ):
        start= time.time()
        ##### The random seed is the same for all the scenarios
        np.random.seed(REPLICAS)
        ##### The Queue is selected
        ActualQueue = Queue[j]()
        print("\n")
        print(type(ActualQueue))
        for i in range(0,len(id2id)):
            #Creating passengers, their arrivals and inserting them to the simulation queue
            x=Passenger(id2id[i][0][0],id2id[i][1][0])
            arrival = PassengerIntoSystem(x,Arriv[0][i])
            ActualQueue.insertBisection(arrival)
        end = time.time()
        print("Creating passengers took "+ str(end-start))

        start=time.time()
        for i in range(0,NumCars):
            # We create drivers and the events that represent their arrivals to the system
            x=Cars[j](TAZList)
            EventArrival = []
            EventLeaving = []
            for k in range(0,len(DriversArrivalEvents[i])):
                # Arrivals
                event = DriverIntoSystem(x,DriversArrivalEvents[i][k])
                ActualQueue.insertBisection(event)
                EventArrival.append(event)

                # Leaving
                event = DriverLeavesSystem(x,DriversLeavingEvents[i][k])
                ActualQueue.insertBisection(event)
                EventLeaving.append(event)

            if(len(EventArrival)>0):
                x.EstablishSchedule(EventArrival,EventLeaving)
        end = time.time()

        ##### For the Single Roaming scenario this information is needed to create the roaming events
        if j==1:
            CarsSingleRandomRoaming.threeZip2osmid = threeZip2osmid
            CarsSingleRandomRoaming.summary_dictionary_all = summary_dictionary_all
            CarsSingleRandomRoaming.final_dict = final_dict
            CarsSingleRandomRoaming.Dict3ZipTo5Zip = Dict3ZipTo5Zip
            CarsSingleRandomRoaming.DictNodeTo3Zip = DictNodeTo3Zip 
            CarsSingleRandomRoaming.hours_intervals = [(((i-1)*hour_in_sec,i*hour_in_sec), tag) if i==1 else
                    (((i-1)*hour_in_sec+1,i*hour_in_sec), tag) for i, tag in hours_list]
            CarsSingleRandomRoaming.days_intervals = [((i*day_in_sec,(i+1)*day_in_sec), tag) if i==0 else 
                    ((i*day_in_sec+1,(i+1)*day_in_sec), tag) for i, tag in days_list]
            CarsSingleRandomRoaming.day_hours_intervals = {j:[((k[0][0]+i[0],k[0][1]+i[0]),k[1]) for k in hours_intervals] for i, j in days_intervals}


        print("Creating drivers took "+ str(end-start))
        print("The size of the queue created is: "+str(len(ActualQueue.elements)))
        print("\n ")
        start= time.time()
        print("Starting to run the simulation number " + str(j) + " replica number " + str(REPLICAS))

        Sim=GeneralSimulator()
        Sim.events=ActualQueue
        ##### These arguments allow to create the driver's roamings from the actual location
        Sim.doAllEvents(SchedulerWithDictionary,TAZDict,FullDict,MapVertexToNode)

        ##### Getting statistics and printing it
        P1 , P2 , P3, P4 = Passenger.GeneralPassengerInformation()
        PassengerNumber[j].append(P1)
        AcceptedDrives[j].append(P2) 
        NonDriverAvailable[j].append(P3)
        DriverFarAway[j].append(P4)

        C1, C2, C3, C4, C5, C6 = Cars[j].GeneralDriversInformation(x)
        CarsNumber[j].append(C1)
        DriverRejecting[j].append(C2)
        DistWithPassengers[j].append(C3)
        DistanceIdle[j].append(C4)
        DistToPas[j].append(C5)
        NumRoamings[j].append(C6)

        ##### Clearing classes' variables for the next simulation run
        Cars[j].ClearDriversInformation(x)
        Passenger.ClearPassengersInformation()
        end = time.time()
        del(ActualQueue) ### deleting altual queue
        print("1 Replica of scenario " + str(j) + " took " + str(end-start) )

    ##### At the end of a replica (the 4 scenarios), results are saved in a pickle file
    Results = []
    Results.append(PassengerNumber)
    Results.append(AcceptedDrives)
    Results.append(NonDriverAvailable)
    Results.append(DriverFarAway)
    Results.append(CarsNumber)
    Results.append(DriverRejecting)
    Results.append(DistWithPassengers)
    Results.append(DistanceIdle)
    Results.append(DistToPas)
    Results.append(NumRoamings)

    NameOfSavedPickle = "StatisticsNoCorrReplica"+str(REPLICAS)+".pickle"
    pickle_disconnected = open(NameOfSavedPickle,"wb")
    pickle.dump(Results,pickle_disconnected)
    pickle_disconnected.close()
