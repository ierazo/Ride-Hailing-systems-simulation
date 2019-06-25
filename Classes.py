# -*- coding: utf-8 -*-
"""
@authors: Ignacio Erazo and Rodrigo de la Fuente
"""
import numpy as np
import igraph
from CreatingGraph import *
import time
from from_to_generator import my_from_to_function
import pickle

"""It must be noted that two different dictionaries are used. In fact, one has the information of all 
the origin-destination pairs possible for passengers and drivers in their roaming events. The other has 
the distance and time from every existent node to the nodes that serve as origin or destination for passengers
or drivers. The main idea is that for usual routes more information is required, while the other dictionary is 
used to compute remaining distances in a roaming event. 
In case you want to use this files it is advisable to check at the way the different queues behave, because
depending on the particularities of your road network, it could be needed to adjust the inputs."""

##### Establishing the maximum waiting time for passengers
MaxWaitingTime = 7*60     ### The maximum waiting time was established as 7 minutes

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


def SchedulerWithDictionary(Passenger,Node,*Drivers,ExtraDist,dictionary):
    DriversAvailable = list(Drivers)
    ExtraDistance = ExtraDist
    #### Checking first if there are drivers available
    if(len(DriversAvailable) == 0) :
        SelectedDriver="No driver is available"
        MinTime=1000
        MinDist = "No driver"
        return SelectedDriver, MinTime, MinDist

    ##### As there are some drivers available, we need to get their shortest path times into a list
    Time = []
    DriversThatCanGo = []
    for i in range(0,len(DriversAvailable)):
        if dictionary[(DriversAvailable[i].node,Node)][0] >= 0:
            ### Getting the shortest path values from the dictionary
            Time.append(dictionary[(DriversAvailable[i].node,Node)][0])
            DriversThatCanGo.append(DriversAvailable[i])

    if ExtraDistance != "No Roaming":
        #### Adding extra distance from the current edge
        for i in range(0,len(Time)):
            Time[i] += ExtraDistance[i]
    AcceptedRide = 0
    while(AcceptedRide==0):
        if(len(DriversThatCanGo) == 0) :
            SelectedDriver="No driver is available"
            MinTime=1000
            MinDist="No Driver"
            return SelectedDriver, MinTime, MinDist
                
        #### Selecting the nearest driver 
        Index = Time.index(min(Time))
        SelectedDriver = DriversThatCanGo[Index]
        if SelectedDriver.Capacity < Passenger.CapacityNeeded or (SelectedDriver.Luxury == False and Passenger.Luxury ==True):
            ### Cannot do the match either because of capacity or luxury constraints
            DriversThatCanGo.pop(Index)
            Time.pop(Index)

        else:
            MinTime = Time[Index]
            valueA=np.random.uniform()
            valueB=ProbToAcceptRide(MinTime,SelectedDriver.MaxTimeToAcceptAlways , SelectedDriver.MaxTimeToRejectAlways)
            #### Driver accepts
            if valueA < valueB:
                ##### Ride is accepted
                MinDist = dictionary[(SelectedDriver.node,Node)][1]
                return SelectedDriver, MinTime, MinDist
            #### Driver rejects
            else:
                #print("Driver rejected the ride because of distance issues")
                type(DriversThatCanGo[Index]).IncreaseRejectedDrives(DriversThatCanGo[Index])
                DriversThatCanGo.pop(Index)
                Time.pop(Index)


##### Used to compute the probability of the driver accepting the ride
def ProbToAcceptRide(Dist, DistSureToAccept, DistSureToReject):
    """This function is used to compute a probability of a given driver to accept a ride, depending on factors
    such as distance between him and the passenger.
    As of now this probability is just a hard rule."""
    if Dist <= DistSureToAccept:
        ProbToAccept = 1
    elif Dist >= DistSureToReject:
        ProbToAccept = 0
    else:
        ProbToAccept = 1 - (Dist - DistSureToAccept) / (DistSureToReject-DistSureToAccept)
    return ProbToAccept


class Event(object):
    """This class is used to initialize all kind of event instances, establishing the way to compare them 
    temporally, in order to create succesfully the simulation queue"""
    def __init__(self):
        self.Time = 0
    def __le__(self,other):
        return self.Time <= other.Time
    def __lt__(self,other):
        return self.Time < other.Time

class DriverIntoSystem(Event):
    """This class is used to characterize when a driver enters the system."""
    def __init__(self,Driver,time):
        self.Time=time
        self.Driver=Driver
        
    def __str__(self):
        result = "Driver " + str(self.Driver.Number) + " arrives at time " + str(self.Time) + " in node" + str(self.Driver.ArrivalNode)
        return result

class DriverLeavesSystem(Event):
    """This class is used to characterize when a driver leaves the system. They have a predefined leaving
    hour, however once they accept a ride, they will leave the system after completing it succesfully."""
    def __init__(self,Driver,time):
        self.Time=time
        self.Driver=Driver
        
    def ChangeLeavingTime(self,Time):
        """ This method is used to change the leaving time, in case the last ride will finish later than the 
         predefined leaving time of the driver """
        self.Time=Time
        
    def __str__(self):
        result = "Driver " + str(self.Driver.Number) + " leaves at time " + str(self.Time) + " in node" + str(self.Driver.node) 
        return result
        
class DriverRoaming(Event):
    def __init__(self,Driver):
        self.Time = Driver.StartRoamingTime + Driver.SPTotalTime
        self.Driver = Driver
        self.DestNode = Driver.DestinationNode

    def __str__(self):
        result = "Driver number "+ str(self.Driver.Number) + " Finishes roaming to node " 
        result += str(self.DestNode) + " at time " + str(self.Time)
        return result


class PassengerIntoSystem(Event):
    """This class is used to characterize when a passenger enters the system. They arrive uniformly in 
    an interval, which is defined in __init__."""
    def __init__(self,Passenger,time):
        self.Time=time
        self.Passenger = Passenger
        
    def __str__(self):
        result = "Passenger " + str(self.Passenger.Number) + " arrives at time " + str(self.Time) 
        result += " in node" + str(self.Passenger.ArrivalNode)
        result+= " wanting to go to node " + str(self.Passenger.DestNode)
        return result
        
class PickUp(Event):
    """This class is used to create the future pick up of the passenger by the respective driver.
    Currently the distance between the driver and the passenger determines the time need for the car
    to arrive to the desired location, being that info given by MinDist parameter."""

    def __init__(self,Event,Passenger,Driver,MinDist):
        self.Time= Event.Time + MinDist
        self.Passenger=Passenger
        self.Driver = Driver          
        
    def __str__(self):
        result = "Passenger " + str(self.Passenger.Number) + " was picked up at time " + str(self.Time) 
        result += " at node" + str(self.Passenger.ArrivalNode)
        result += " by driver" + str(self.Driver.Number)
        return result
    
class Dropoff(Event):
    """This class is used to create the future dropoff of the passenger by the respective driver.
    Currently the distance between the destination node and the passenger arrival node
    determines the time needed to arrive to the desired location. That information is given by parameter MinDist."""
    def __init__(self,Event,Passenger,Driver,MinDist):
        self.Time = Event.Time + MinDist
        self.Passenger = Passenger
        self.Driver = Driver
                
    def __str__(self):
        result = "Passenger " + str(self.Passenger.Number) + " was dropped at time " + str(self.Time) 
        result += " in node" + str(self.Passenger.DestNode) 
        result += " by driver " + str(self.Driver.Number)
        return result
    

class DriversInSystem(object):
    """This class is just a list with all the available drivers at any moment. It gets updated when 
    assignments are made, or when dropoff events are concluded. 
    
    It is needed to test the efficiency of this method, changing it to another idea like conditionals, or 
    inserting it at the latest position instead of the first."""
    def __init__(self):
        self.elements=list()
    def insert(self,Driver):
        self.elements.insert(0,Driver)  ### Insert proved to be faster than append, that is why it is used
    def remove(self,Driver):
        self.elements.remove(Driver)

class Passenger(object):
    """This class represents passengers, initializing them with an initial location and a destination.
    They will be passed as input to an event that makes them arrive to the system."""

    ##### Class attributes
    __PassengerNumber = 0
    __AcceptedDrives = 0
    __NonDriverAvailable = 0
    __DriverFarAway = 0

    def __init__(self,ArrivalNode,DestinationNode):  
        """Here the random nodes are assigned"""
        self.ArrivalNode = ArrivalNode
        self.DestNode = DestinationNode
        type(self).__PassengerNumber += 1
        self.Number = type(self).__PassengerNumber
        self.MaxWaitingTime = MaxWaitingTime

        ####Capacity conditions
        a = np.random.uniform()
        if a<0.9:
            self.CapacityNeeded = np.random.randint(1,5)
        else:
            self.CapacityNeeded = np.random.randint(5,7)
        #### Luxury conditions
        a = np.random.uniform()
        if a<0.94:
            self.Luxury = False
        else:
            self.Luxury = True

    ##### Methods to get statistics about the simulation
    @staticmethod
    def IncreaseAcceptedDrives():
        Passenger.__AcceptedDrives +=1
        return

    @staticmethod
    def IncreaseNonDriverAvailable():
        Passenger.__NonDriverAvailable +=1
        return

    @staticmethod
    def IncreaseDriverFarAway():
        Passenger.__DriverFarAway += 1
        return

    @staticmethod
    def GeneralPassengerInformation():
        print("The number of passengers is " + str(Passenger.__PassengerNumber))
        print("The number of accepted drives is " + str(Passenger.__AcceptedDrives))
        print("The number of drives rejected because no driver was available is " + str(Passenger.__NonDriverAvailable))
        print("The number of drives non accepted because the driver was far away is " + str(Passenger.__DriverFarAway))
        return Passenger.__PassengerNumber , Passenger.__AcceptedDrives , Passenger.__NonDriverAvailable, Passenger.__DriverFarAway

    @staticmethod
    def ClearPassengersInformation():
        Passenger.__PassengerNumber = 0
        Passenger.__AcceptedDrives = 0
        Passenger.__NonDriverAvailable = 0
        Passenger.__DriverFarAway = 0

    def __str__(self):
        return 

class CarsNoRoaming(object):
    """This class represent the drivers.
    They are given as input to some entering event (which needs information about which driver enters the system)
    which means they will enter the system depending on the structure defined in the entering event. They have
    some leaving event too, with a predefined but modifiable leaving time.
    These drivers only move when they need to go pick up a passenger and dropoff them. They change their node
    to the destination node and then wait there (they do not hunt for passengers)"""
    
    CarsNumbers = 0
    DistanceDrivenWithPassengers = 0
    DistanceDrivenToPassengers = 0
    DistanceDrivenIdle = 0
    DriverRejectingDrives = 0
    NumberOfRoamings = 0

    def __init__(self,ListTAZ):
        #### Here the Number of the car is defined
        type(self).CarsNumbers+=1
        self.Number=type(self).CarsNumbers

        #### Establishing the time that drivers may be willing to drive in order to PickUp a passenger
        self.MaxTimeToAcceptAlways = np.random.uniform(2*60,4*60)
        self.MaxTimeToRejectAlways = np.random.uniform(19*60,21*60)

        self.EstablishCapacity()
        self.EstablishLuxury()
        self.EstablishArrivalLocation(ListTAZ)
        self.node = self.ArrivalNode

    def EstablishLuxury(self):
        a = np.random.uniform(0,1)
        if a < 0.9:
            self.Luxury = False
        else:
            self.Luxury = True

    def EstablishCapacity(self):
        """ Establishing the capacity allowed for each car """
        a = np.random.uniform()
        if a < (1-0.164):   ### Just a normal car, parameter obtained with real data
            self.Capacity = np.random.randint(3,5)
        else:
            self.Capacity = np.random.randint(5,7)

    def EstablishSchedule(self,ArrivalEvents,LeavingEvents):
        self.ArrivalEvents = ArrivalEvents  
        self.LeavingEvents = LeavingEvents
        self.LeavingHour = LeavingEvents[0].Time
        self.DesiredLeavingHour = self.LeavingHour  ### This condition is only used in roaming procedures

    def EstablishArrivalLocation(self,ListTAZ):
        ##### A random TAZ is selected
        value = np.random.randint(0,len(ListTAZ))
        ##### Here the node is obtained
        self.ArrivalNode = ListTAZ[value]

    @staticmethod
    def IncreaseDistancePassengers(self,MinDist):
        type(self).DistanceDrivenWithPassengers+=MinDist
        return

    @staticmethod
    def IncreaseDistanceIdle(self,MinDist):
        type(self).DistanceDrivenIdle+=MinDist
        return

    @staticmethod
    def IncreaseDistanceToPassengers(self,MinDist):
        type(self).DistanceDrivenToPassengers+=MinDist
        return

    @staticmethod
    def IncreaseRejectedDrives(self):
        type(self).DriverRejectingDrives += 1

    @staticmethod
    def GeneralDriversInformation(self):
        print("The total number of drivers is " + str(type(self).CarsNumbers))
        print("The total number of drives non acepted by drivers is " +str(type(self).DriverRejectingDrives))
        print("The total distance driven with passengers is " + str(type(self).DistanceDrivenWithPassengers))
        print("The total distance driven idle " + str(type(self).DistanceDrivenIdle))
        print("The total distance driven going to pick up passengers is " + str(type(self).DistanceDrivenToPassengers))
        print("The total number of roamings started by drivers was: " + str(type(self).NumberOfRoamings) )
        return type(self).CarsNumbers , type(self).DriverRejectingDrives , type(self).DistanceDrivenWithPassengers, type(self).DistanceDrivenIdle , type(self).DistanceDrivenToPassengers , type(self).NumberOfRoamings

    @staticmethod
    def ClearDriversInformation(self):
        type(self).CarsNumbers = 0
        type(self).DistanceDrivenWithPassengers = 0
        type(self).DistanceDrivenToPassengers = 0
        type(self).DistanceDrivenIdle = 0
        type(self).DriverRejectingDrives = 0
        type(self).NumberOfRoamings = 0

    def NewLeavingEvent(self,Event):
        self.LeavingEvents[0] = Event
        self.LeavingHour = Event.Time
        self.Time = Event.Time
        
    def ChangePosition(self,NewNode):
        self.node = NewNode

    def __str__(self):
        result = "Driver " + str(self.Number) + " is at node " + str(self.node)
        return result

class CarsSpecificHotspot(CarsNoRoaming):
    """Here drivers go to a specific location given by the ride-hailing company"""
    
    CarsNumbers = 0
    DistanceDrivenWithPassengers = 0
    DistanceDrivenToPassengers = 0
    DistanceDrivenIdle = 0
    DriverRejectingDrives = 0
    NumberOfRoamings = 0
    Hotspots = []
    RoutedCarsToHotspots = []
    PrioritiesHotspots = []
    ### An object attribute called Cars.Hotspots will be added

    def __init__(self,ListTAZ):
        #### Here the Number of the car is defined
        type(self).CarsNumbers+=1
        self.Number=type(self).CarsNumbers

        #### Establishing the time that drivers may be willing to drive in order to PickUp a passenger
        self.MaxTimeToAcceptAlways = np.random.uniform(2*60,4*60)
        self.MaxTimeToRejectAlways = np.random.uniform(19*60,21*60)

        self.EstablishCapacity()
        self.EstablishLuxury()
        self.EstablishArrivalLocation(ListTAZ)
        self.node = self.ArrivalNode
        ##### These are used toestablish accurately the working conditions of drivers
        self.PseudoEliminatedArrivalEvents = []
        self.PseudoEliminatedLeavingEvents = []
        self.NodesInGraph = len(ListTAZ)
        self.ListTAZ = ListTAZ


    def CreateRoaming(self,dictionary,time):
        self.PartialNodeIndex = 0
        self.StartRoamingTime = time
        self.RoamingFinished = False
        type(self).NumberOfRoamings += 1
        
        self.SPTimes, self.SPDistances , self.SPEdges ,self.SPVertices, self.SPTotalTime, self.TotalDist = self.ShortestPathRoaming(dictionary) 


    def NewRoamingEvent(self,Event):
        self.RoamingEvent = Event

    def ShortestPathRoaming(self,dictionary):
        PossibleHotspots = []
        Priorities = []
        self.RoamingEliminated = 0
        ###### Getting information about the number of routed cars for each hotspot
        MinimumVal = min(type(self).RoutedCarsToHotspots)
        if MinimumVal < 2: #### This value could be changed, it is a parameter
            ###### At least 1 hotspot has less than 2 cars routed there, so we will restrict the analysis to those hotspots
            for i in range(0,len(type(self).RoutedCarsToHotspots)):
                if type(self).RoutedCarsToHotspots[i]==MinimumVal:
                    PossibleHotspots.append(type(self).Hotspots[i])
                    Priorities.append(type(self).PrioritiesHotspots[i])
        else: 
            ####### otherwise all hotspots are evaluated
            PossibleHotspots = type(self).Hotspots[:]
            Priorities = type(self).PrioritiesHotspots[:]
        ShortestPathEdges = []
        Distances = []
        for j in PossibleHotspots:
            Distances.append(dictionary[(self.node,j)][4])
        
        Prioridad = []
        for i in range(0,len(Distances)):
            Prioridad.append(Priorities[i]/(Distances[i]+1))

        #Getting the highest value of total priority
        Index = Prioridad.index(max(Prioridad))
        self.DestinationNode = PossibleHotspots[Index]

        ##### Getting all the required info from the dictionary
        ShortestPathEdges = dictionary[(self.node,self.DestinationNode)][2]
        ShortestPathVertices = dictionary[(self.node,self.DestinationNode)][3]
        ShortestPathTimes = dictionary[(self.node,self.DestinationNode)][0]
        ShortestPathDistances = dictionary[(self.node,self.DestinationNode)][1]
        ShortestPathTotalTimes=dictionary[(self.node,self.DestinationNode)][4]
        TotalDist = dictionary[(self.node,self.DestinationNode)][5]

        ##### Updating the number of routed cars to each hotspot
        Index = type(self).Hotspots.index(self.DestinationNode)
        type(self).RoutedCarsToHotspots[Index]+=1

        return ShortestPathTimes, ShortestPathDistances, ShortestPathEdges, ShortestPathVertices, ShortestPathTotalTimes, TotalDist

    
        
class CarsNearestHotspot(CarsSpecificHotspot):

    CarsNumbers = 0
    DistanceDrivenWithPassengers = 0
    DistanceDrivenToPassengers = 0
    DistanceDrivenIdle = 0
    DriverRejectingDrives = 0
    NumberOfRoamings = 0
    Hotspots = []

    def ShortestPathRoaming(self,dictionary):
        ##### Getting distances to all of the hotspots
        Distances = []
        for j in (type(self).Hotspots):
            Distances.append(dictionary[self.node,j][4])
        ##### Selecting the nearest hotspot
        Index = Distances.index(min(Distances))
        self.DestinationNode = type(self).Hotspots[Index]
        ##### Getting the required information from the dictionary used
        ShortestPathEdges = dictionary[(self.node,self.DestinationNode)][2]
        ShortestPathVertices = dictionary[(self.node,self.DestinationNode)][3]
        ShortestPathTimes = dictionary[(self.node,self.DestinationNode)][0]
        ShortestPathDistances = dictionary[(self.node,self.DestinationNode)][1]
        ShortestPathTotalTimes=dictionary[(self.node,self.DestinationNode)][4]
        TotalDist = dictionary[(self.node,self.DestinationNode)][5]

        return ShortestPathTimes, ShortestPathDistances, ShortestPathEdges, ShortestPathVertices, ShortestPathTotalTimes, TotalDist

class CarsSingleRandomRoaming(CarsSpecificHotspot):
    CarsNumbers = 0
    DistanceDrivenWithPassengers = 0
    DistanceDrivenToPassengers = 0
    DistanceDrivenIdle = 0
    DriverRejectingDrives = 0
    NumberOfRoamings = 0

    def ShortestPathRoaming(self,dictionary):
        if self.StartRoamingTime < 7*24*3600: ##### one week (in seconds) is simulated
            a,b =my_from_to_function(self.StartRoamingTime, CarsSingleRandomRoaming.day_hours_intervals, 
                    CarsSingleRandomRoaming.days_intervals, hourly_blocks,
                    CarsSingleRandomRoaming.threeZip2osmid, CarsSingleRandomRoaming.summary_dictionary_all,
                    CarsSingleRandomRoaming.final_dict,
                    start5zip=CarsSingleRandomRoaming.Dict3ZipTo5Zip[CarsSingleRandomRoaming.DictNodeTo3Zip[self.node]],
                    status='idle', verbose=False)
            ##### The roaming destiantion depends on the taxi dataset and the hour
            self.DestinationNode = a[1][0]
        else: ##### We passed the simulation time so the my_from_to function does not work
            a = np.random.randint(0,self.NodesInGraph)
            self.DestinationNode = self.ListTAZ[a]

        while self.node == self.DestinationNode: ##### We repeat the procedure from above if the driver
            ##### does not want to roam.
            if self.StartRoamingTime < 7*24*3600:
                a,b =my_from_to_function(self.StartRoamingTime, CarsSingleRandomRoaming.day_hours_intervals, 
                    CarsSingleRandomRoaming.days_intervals, hourly_blocks,
                    CarsSingleRandomRoaming.threeZip2osmid, CarsSingleRandomRoaming.summary_dictionary_all,
                    CarsSingleRandomRoaming.final_dict,
                    start5zip=CarsSingleRandomRoaming.Dict3ZipTo5Zip[CarsSingleRandomRoaming.DictNodeTo3Zip[self.node]],
                    status='idle', verbose=False)
                self.DestinationNode = a[1][0]
            else:
                a = np.random.randint(0,self.NodesInGraph)
                self.DestinationNode = self.ListTAZ[a]

        ##### Getting information from the dictionary
        ShortestPathEdges = dictionary[(self.node,self.DestinationNode)][2]
        ShortestPathVertices = dictionary[(self.node,self.DestinationNode)][3]
        ShortestPathTimes = dictionary[(self.node,self.DestinationNode)][0]
        ShortestPathDistances = dictionary[(self.node,self.DestinationNode)][1]
        ShortestPathTotalTimes=dictionary[(self.node,self.DestinationNode)][4]
        TotalDist = dictionary[(self.node,self.DestinationNode)][5]
        
        return ShortestPathTimes, ShortestPathDistances, ShortestPathEdges, ShortestPathVertices, ShortestPathTotalTimes, TotalDist


class ListQueue(object):
    """This class represents the simulation queue. Events are inserted depending on their execution time.
    Events are removed once the simulation executes them (chronologically). 
    Depending on the type of event, another may be created and scheduler. As an example, a pick-up event will 
    trigger the creation of a dropoff event.
    """
    elements = list()

    def insertBisection(self,x):
        '''Inserts event x in the position i, by using a bisection algorithm'''
        if len(self.elements)==0:
            self.elements.insert(0,x)
            return
        else:
            a=0
            c=len(self.elements)-1
            b = int((a+c)/2)
            if x < self.elements[0]:
                self.elements.insert(0,x)
                return
            elif self.elements[c] <= x:
                self.elements.insert(c+1,x)
                return

            while(a<b):
                if self.elements[b]<=x:
                    a=b
                    b=int((a+c)/2)
                else:
                    c=b
                    b=int((a+c)/2)
            self.elements.insert(a+1,x)
            return

    def remove(self,x):
        '''Helps to remove an event that will happen in the future'''
        for i in range(len(self.elements)):
            if self.elements[i] == x:
                return self.elements.pop(i)
        return None

    def size(self):
        '''May be used to know the size of the events queue'''
        return len(self.elements)
    
    def __str__(self):
        result=''
        for i in range(0,len(self.elements)):
            result = result + str(self.elements[i]) + " \n "
        return result

class ListQueueNoRoaming(ListQueue):

    def removeFirst(self,DriversInSystem,SchedulerType,ShortestPathDictionary,FullDict,MapVertexToNode):
        '''Removes the event that is happening'''
        if len(self.elements) == 0:
            return None
        
        if isinstance(self.elements[0],DriverIntoSystem) :
            ##### Add the driver to the list of available drivers
            DriversInSystem.insert(self.elements[0].Driver)
            ##### Establish his location
            self.elements[0].Driver.node = self.elements[0].Driver.ArrivalNode
            ##### Eliminate the arrival event
            self.elements[0].Driver.ArrivalEvents.pop(0)
            x = self.elements.pop(0)
            return x
        
        if isinstance(self.elements[0], DriverLeavesSystem) :
            ##### Remove driver from the list of available drivers
            DriversInSystem.remove(self.elements[0].Driver)
            ##### Eliminate the leaving event
            self.elements[0].Driver.LeavingEvents.pop(0)
            ##### If he enters and leaves the system again, establish his next leaving time
            if len(self.elements[0].Driver.LeavingEvents) > 0:
                self.elements[0].Driver.LeavingHour = self.elements[0].Driver.LeavingEvents[0].Time
            x = self.elements.pop(0)
            return x
        
        if isinstance(self.elements[0], PassengerIntoSystem) :
            ExtraDist = "No Roaming"
            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected=="No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                x = self.elements.pop(0)
                return x

            elif MinTime > self.elements[0].Passenger.MaxWaitingTime:
                Passenger.IncreaseDriverFarAway()
                #print("The ride was not accepted by the customer because it exceeded his maximum waiting time")
                x = self.elements.pop(0)
                return x
            else:
                ########                The drive is accepted
                Passenger.IncreaseAcceptedDrives()
                ########                 Creating the pickup event generated by this drive
                NewPickUp = PickUp(self.elements[0],self.elements[0].Passenger,DriverSelected,MinTime)
                self.insertBisection(NewPickUp)
                ########                 Increase total distance that will be driven
                DriverSelected.IncreaseDistanceToPassengers(DriverSelected,MinDist)
                ########                 Remove the driver from the available list
                DriversInSystem.remove(DriverSelected)
                ########       If the ride exceeds the driver leaving time, a new leaving event for the driver is created
                if DriverSelected.LeavingHour<NewPickUp.Time:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents[0].ChangeLeavingTime(NewPickUp.Time+0.001)
                    DriverSelected.LeavingHour = NewPickUp.Time+0.001
                    self.insertBisection(DriverSelected.LeavingEvents[0])
                    while (len(DriverSelected.ArrivalEvents) > 0 and DriverSelected.ArrivalEvents[0].Time <= DriverSelected.LeavingEvents[0].Time):
                        self.remove(DriverSelected.ArrivalEvents[0])
                        self.remove(DriverSelected.LeavingEvents[1])
                        DriverSelected.ArrivalEvents.pop(0)
                        DriverSelected.LeavingEvents.pop(1)
                #print("driver chosen is " + str(DriverSelected.Number) + "at time " + str(self.elements[0].Time) +" his time to passenger is: " + str(MinTime) + "his dist is " + str(MinDist))
                x = self.elements.pop(0)
                return x
        
        if isinstance(self.elements[0],PickUp):
            ###        First we update the new position of the driver
            self.elements[0].Driver.ChangePosition(self.elements[0].Passenger.ArrivalNode)
            ###         Now we compute the shortest path between passenger arrival location and its destination
            MinTime = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][4] 
            MinDist = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][5] 
            ###        We create the dropoff event considering the time it will take to arrive to the destination
            NewDropoffEvent=Dropoff(self.elements[0],self.elements[0].Passenger,self.elements[0].Driver,MinTime)
            self.insertBisection(NewDropoffEvent)
            ###          Increase total distance that will be driven
            self.elements[0].Driver.IncreaseDistancePassengers(self.elements[0].Driver,MinDist)

            ####        If the ride exceeds the driver leaving time, a new leaving event for the driver is created
            if NewDropoffEvent.Driver.LeavingHour < NewDropoffEvent.Time:
                self.remove(NewDropoffEvent.Driver.LeavingEvents[0])
                NewDropoffEvent.Driver.LeavingEvents[0].ChangeLeavingTime(NewDropoffEvent.Time+0.001)
                NewDropoffEvent.Driver.LeavingHour = NewDropoffEvent.Time+0.001
                self.insertBisection(NewDropoffEvent.Driver.LeavingEvents[0])
                while(len(NewDropoffEvent.Driver.ArrivalEvents) > 0 and NewDropoffEvent.Driver.ArrivalEvents[0].Time <= NewDropoffEvent.Driver.LeavingEvents[0].Time):
                    self.remove(NewDropoffEvent.Driver.ArrivalEvents[0])
                    self.remove(NewDropoffEvent.Driver.LeavingEvents[1])
                    NewDropoffEvent.Driver.ArrivalEvents.pop(0)
                    NewDropoffEvent.Driver.LeavingEvents.pop(1)

            #print(str(self.elements[0])+ " time to go to the destination is " +str(MinTime)+" dist to go is " + str(MinDist))
            x = self.elements.pop(0)
            return x
        
        if isinstance(self.elements[0],Dropoff):
            ####          The position of the driver is updated
            self.elements[0].Driver.ChangePosition(self.elements[0].Passenger.DestNode)
            ####           The driver is added to the list of available drivers 
            DriversInSystem.insert(self.elements[0].Driver)
            #print(str(self.elements[0]))
            x=self.elements.pop(0)
            return x
            

class ListQueueSpecificHotspot(ListQueue):

    """This class represents the simulation queue. Events are inserted depending on their execution time.
    Events are removed once the simulation executes them (chronologically). 
    Depending on the type of event, another may be created and scheduler. As an example, a pick-up event will 
    trigger the creation of a dropoff event.
    """

    def removeFirst(self,DriversInSystem,SchedulerType,ShortestPathDictionary,FullDict,MapVertexToNode):
        '''Removes the event that is happening'''
        if len(self.elements) == 0:
            return None
        
        if isinstance(self.elements[0],DriverIntoSystem) :
            d= self.elements[0].Driver
            DriversInSystem.insert(d)
            d.node = d.ArrivalNode
            d.ArrivalEvents.pop(0)

            #Creating a roaming Event
            d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
            E = DriverRoaming(d)
            d.NewRoamingEvent(E)
            self.insertBisection(E)
            ##### Adjust leaving time in case the evnt exceeds the tentative leaving time
            if d.LeavingHour <= E.Time:
                self.remove(d.LeavingEvents[0])
                d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                d.LeavingEvents.pop(0)
                NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                d.LeavingHour = E.Time+0.001
                self.insertBisection(NewLeavingEvent)
                while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                    self.remove(d.ArrivalEvents[0])
                    d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                    d.ArrivalEvents.pop(0)
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                d.LeavingEvents.insert(0,NewLeavingEvent)
            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0], DriverLeavesSystem) :
            DriversInSystem.remove(self.elements[0].Driver)

            if self.elements[0].Driver.RoamingEliminated == 0:
                Index = type(self.elements[0].Driver).Hotspots.index(self.elements[0].Driver.DestinationNode)
                type(self.elements[0].Driver).RoutedCarsToHotspots[Index]-= 1 
            self.elements[0].Driver.LeavingEvents.pop(0)

            if len(self.elements[0].Driver.LeavingEvents) > 0:
                self.elements[0].Driver.LeavingHour = self.elements[0].Driver.LeavingEvents[0].Time
                self.elements[0].Driver.DesiredLeavingHour  = self.elements[0].Driver.LeavingEvents[0].Time
            self.elements[0].Driver.PseudoEliminatedArrivalEvents = []
            self.elements[0].Driver.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x


        if isinstance(self.elements[0], DriverRoaming):
            ### The roaming has just finished. The driver will go roaming somewhere else ONLY if it has not surpassed his leaving time
            d=self.elements[0].Driver
            ### Before updating the new position of the driver, we will collect statistics:
            type(d).IncreaseDistanceIdle(d,d.TotalDist)
            d.ChangePosition(d.DestinationNode)
            d.RoamingFinished = True
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0], PassengerIntoSystem):
            ### Update all the drivers that are roaming:
            ExtraDist = []
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        ##### Getting the actual position of the driver
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))

            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                x = self.elements.pop(0)
                return x
            elif MinTime > self.elements[0].Passenger.MaxWaitingTime:
                Passenger.IncreaseDriverFarAway()
                x = self.elements.pop(0)
                return x
            else:
                ########                The drive is accepted
                Passenger.IncreaseAcceptedDrives()
                ######## We must update the driven distances:
                if DriverSelected.RoamingFinished == False:
                    DistIdle = DriverSelected.SPDistances[DriverSelected.PartialNodeIndex]
                    type(DriverSelected).IncreaseDistanceIdle(DriverSelected,DistIdle)
                    self.remove(DriverSelected.RoamingEvent)
                ########                 Creating the pickup event generated by this drive
                NewPickUp = PickUp(self.elements[0],self.elements[0].Passenger,DriverSelected,MinTime)
                self.insertBisection(NewPickUp)
                ########                 Increase total distance that will be driven
                type(DriverSelected).IncreaseDistanceToPassengers(DriverSelected,MinDist)
                ########                 Remove the driver from the available list
                DriversInSystem.remove(DriverSelected)
                ########                 Remove the car of the list that has cars roamed to specific hotspots
                Index = type(DriverSelected).Hotspots.index(DriverSelected.DestinationNode)
                if DriverSelected.RoamingEliminated == 0:
                    type(DriverSelected).RoutedCarsToHotspots[Index]-= 1 
                    DriverSelected.RoamingEliminated = 1
                DriverSelected.RoamingFinished = True
                ########       If the ride exceeds the driver leaving time, a new leaving event for the driver is created
                if len(DriverSelected.PseudoEliminatedLeavingEvents) > 0:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents.pop(0)
                    for i in range(0,len(DriverSelected.PseudoEliminatedLeavingEvents)):
                        DriverSelected.LeavingEvents.insert(0,DriverSelected.PseudoEliminatedLeavingEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.LeavingEvents[0])
                    DriverSelected.PseudoEliminatedLeavingEvents = []

                    DriverSelected.LeavingHour = DriverSelected.LeavingEvents[0].Time

                    for i in range(0,len(DriverSelected.PseudoEliminatedArrivalEvents)):
                        DriverSelected.ArrivalEvents.insert(0,DriverSelected.PseudoEliminatedArrivalEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.ArrivalEvents[0])

                if DriverSelected.LeavingHour<NewPickUp.Time:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents[0].ChangeLeavingTime(NewPickUp.Time+0.001)
                    DriverSelected.LeavingHour = NewPickUp.Time+0.001
                    self.insertBisection(DriverSelected.LeavingEvents[0])
                    while (len(DriverSelected.ArrivalEvents) > 0 and DriverSelected.ArrivalEvents[0].Time <= DriverSelected.LeavingEvents[0].Time):
                        self.remove(DriverSelected.ArrivalEvents[0])
                        self.remove(DriverSelected.LeavingEvents[1])
                        DriverSelected.ArrivalEvents.pop(0)
                        DriverSelected.LeavingEvents.pop(1)
                x = self.elements.pop(0)
                return x
                

        
        if isinstance(self.elements[0],PickUp):
            ###        First we update the new position of the driver
            self.elements[0].Driver.ChangePosition(self.elements[0].Passenger.ArrivalNode)
            ###         Now we compute the shortest path between passenger arrival location and its destination
            MinTime = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][4] 
            MinDist = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][5] 
            ###        We create the dropoff event considering the time it will take to arrive to the destination
            NewDropoffEvent=Dropoff(self.elements[0],self.elements[0].Passenger,self.elements[0].Driver,MinTime)
            self.insertBisection(NewDropoffEvent)
            ###          Increase total distance that will be driven
            type(self.elements[0].Driver).IncreaseDistancePassengers(self.elements[0].Driver,MinDist)

            ####        If the ride exceeds the driver leaving time, a new leaving event for the driver is created
            if NewDropoffEvent.Driver.LeavingHour < NewDropoffEvent.Time:
                self.remove(NewDropoffEvent.Driver.LeavingEvents[0])
                NewDropoffEvent.Driver.LeavingEvents[0].ChangeLeavingTime(NewDropoffEvent.Time+0.001)
                NewDropoffEvent.Driver.LeavingHour = NewDropoffEvent.Time+0.001
                self.insertBisection(NewDropoffEvent.Driver.LeavingEvents[0])
                while(len(NewDropoffEvent.Driver.ArrivalEvents) > 0 and NewDropoffEvent.Driver.ArrivalEvents[0].Time <= NewDropoffEvent.Driver.LeavingEvents[0].Time):
                    self.remove(NewDropoffEvent.Driver.ArrivalEvents[0])
                    self.remove(NewDropoffEvent.Driver.LeavingEvents[1])
                    NewDropoffEvent.Driver.ArrivalEvents.pop(0)
                    NewDropoffEvent.Driver.LeavingEvents.pop(1)

            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0],Dropoff):
            ####          The position of the driver is updated
            d = self.elements[0].Driver
            d.ChangePosition(self.elements[0].Passenger.DestNode)
            ####           The driver is added to the list of available drivers 
            DriversInSystem.insert(d)
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            if d.DesiredLeavingHour > self.elements[0].Time:
                d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
                E = DriverRoaming(d)
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                if d.LeavingHour <= E.Time:
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                    NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                    d.LeavingHour = E.Time+0.001
                    self.insertBisection(NewLeavingEvent)

                    while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                        self.remove(d.ArrivalEvents[0])
                        d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                        d.ArrivalEvents.pop(0)
                        self.remove(d.LeavingEvents[0])
                        d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                        d.LeavingEvents.pop(0)
                    d.LeavingEvents.insert(0,NewLeavingEvent)
            else: 
                d.RoamingFinished = True

            x=self.elements.pop(0)
            return x
            

class ListQueueNearestHotspot(ListQueue):

    def removeFirst(self,DriversInSystem,SchedulerType,ShortestPathDictionary,FullDict,MapVertexToNode):
        '''Removes the event that is happening'''
        if len(self.elements) == 0:
            return None

        if isinstance(self.elements[0],DriverIntoSystem) :
            d= self.elements[0].Driver
            DriversInSystem.insert(d)
            d.node = d.ArrivalNode
            d.ArrivalEvents.pop(0)
            #Creating a roaming Event
            d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
            E = DriverRoaming(d)
            d.NewRoamingEvent(E)
            self.insertBisection(E)
            if d.LeavingHour <= E.Time:
                self.remove(d.LeavingEvents[0])
                d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                d.LeavingEvents.pop(0)
                NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                d.LeavingHour = E.Time+0.001
                self.insertBisection(NewLeavingEvent)
                while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                    self.remove(d.ArrivalEvents[0])
                    d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                    d.ArrivalEvents.pop(0)
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                d.LeavingEvents.insert(0,NewLeavingEvent)
            x = self.elements.pop(0)
            return x


        if isinstance(self.elements[0], DriverLeavesSystem) :
            DriversInSystem.remove(self.elements[0].Driver)
            self.elements[0].Driver.LeavingEvents.pop(0)
            if len(self.elements[0].Driver.LeavingEvents) > 0:
                self.elements[0].Driver.LeavingHour = self.elements[0].Driver.LeavingEvents[0].Time
                self.elements[0].Driver.DesiredLeavingHour  = self.elements[0].Driver.LeavingEvents[0].Time
            self.elements[0].Driver.PseudoEliminatedArrivalEvents = []
            self.elements[0].Driver.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x



        if isinstance(self.elements[0], DriverRoaming):
            ### The roaming has just finished. The driver will go roaming somewhere else ONLY if it has not surpassed his leaving time
            d=self.elements[0].Driver
            ### Before updating the new position of the driver, we will collect statistics:
            type(d).IncreaseDistanceIdle(d,d.TotalDist)
            d.ChangePosition(d.DestinationNode)
            d.RoamingFinished = True
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0], PassengerIntoSystem):
            ### Update all the drivers that are roaming:
            ExtraDist = []
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))
            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                x = self.elements.pop(0)
                return x
            elif MinTime > self.elements[0].Passenger.MaxWaitingTime:
                Passenger.IncreaseDriverFarAway()
                x = self.elements.pop(0)
                return x
            else:
                ########                The drive is accepted
                Passenger.IncreaseAcceptedDrives()
                ######## We must update the driven distances:
                if DriverSelected.RoamingFinished == False:
                    DistIdle = DriverSelected.SPDistances[DriverSelected.PartialNodeIndex]
                    type(DriverSelected).IncreaseDistanceIdle(DriverSelected,DistIdle)
                ########                 Creating the pickup event generated by this drive
                NewPickUp = PickUp(self.elements[0],self.elements[0].Passenger,DriverSelected,MinTime)
                self.insertBisection(NewPickUp)
                self.remove(DriverSelected.RoamingEvent)
                ########                 Increase total distance that will be driven
                type(DriverSelected).IncreaseDistanceToPassengers(DriverSelected,MinDist)
                ########                 Remove the driver from the available list
                DriversInSystem.remove(DriverSelected)
                DriverSelected.RoamingFinished = True

                if len(DriverSelected.PseudoEliminatedLeavingEvents) > 0:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents.pop(0)
                    for i in range(0,len(DriverSelected.PseudoEliminatedLeavingEvents)):
                        DriverSelected.LeavingEvents.insert(0,DriverSelected.PseudoEliminatedLeavingEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.LeavingEvents[0])
                    DriverSelected.PseudoEliminatedLeavingEvents = []

                    DriverSelected.LeavingHour = DriverSelected.LeavingEvents[0].Time

                    for i in range(0,len(DriverSelected.PseudoEliminatedArrivalEvents)):
                        DriverSelected.ArrivalEvents.insert(0,DriverSelected.PseudoEliminatedArrivalEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.ArrivalEvents[0])

                if DriverSelected.LeavingHour<NewPickUp.Time:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents[0].ChangeLeavingTime(NewPickUp.Time+0.001)
                    DriverSelected.LeavingHour = NewPickUp.Time+0.001
                    self.insertBisection(DriverSelected.LeavingEvents[0])
                    while (len(DriverSelected.ArrivalEvents) > 0 and DriverSelected.ArrivalEvents[0].Time <= DriverSelected.LeavingHour):
                        self.remove(DriverSelected.ArrivalEvents[0])
                        self.remove(DriverSelected.LeavingEvents[1])
                        DriverSelected.ArrivalEvents.pop(0)
                        DriverSelected.LeavingEvents.pop(1)
                x = self.elements.pop(0)
                return x


        if isinstance(self.elements[0],PickUp):
            ###        First we update the new position of the driver
            self.elements[0].Driver.ChangePosition(self.elements[0].Passenger.ArrivalNode)
            ###         Now we compute the shortest path between passenger arrival location and its destination
            MinTime = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][4] 
            MinDist = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][5]
            ###        We create the dropoff event considering the time it will take to arrive to the destination
            NewDropoffEvent=Dropoff(self.elements[0],self.elements[0].Passenger,self.elements[0].Driver,MinTime)
            self.insertBisection(NewDropoffEvent)
            ###          Increase total distance that will be driven
            type(self.elements[0].Driver).IncreaseDistancePassengers(self.elements[0].Driver,MinDist)
            ####        If the ride exceeds the driver leaving time, a new leaving event for the driver is created
            if NewDropoffEvent.Driver.LeavingHour < NewDropoffEvent.Time:
                self.remove(NewDropoffEvent.Driver.LeavingEvents[0])
                NewDropoffEvent.Driver.LeavingEvents[0].ChangeLeavingTime(NewDropoffEvent.Time+0.001)
                NewDropoffEvent.Driver.LeavingHour = NewDropoffEvent.Time+0.001
                self.insertBisection(NewDropoffEvent.Driver.LeavingEvents[0])
                while(len(NewDropoffEvent.Driver.ArrivalEvents) > 0 and NewDropoffEvent.Driver.ArrivalEvents[0].Time <= NewDropoffEvent.Driver.LeavingEvents[0].Time):
                    self.remove(NewDropoffEvent.Driver.ArrivalEvents[0])
                    self.remove(NewDropoffEvent.Driver.LeavingEvents[1])
                    NewDropoffEvent.Driver.ArrivalEvents.pop(0)
                    NewDropoffEvent.Driver.LeavingEvents.pop(1)

            x = self.elements.pop(0)
            return x


        if isinstance(self.elements[0],Dropoff):
            ####          The position of the driver is updated
            d = self.elements[0].Driver
            d.ChangePosition(self.elements[0].Passenger.DestNode)
            ####           The driver is added to the list of available drivers 
            DriversInSystem.insert(d)
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            if d.DesiredLeavingHour > self.elements[0].Time:
                d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
                E = DriverRoaming(d)
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                if d.LeavingHour <= E.Time:
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                    NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                    d.LeavingHour = E.Time+0.001
                    self.insertBisection(NewLeavingEvent)

                    while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                        self.remove(d.ArrivalEvents[0])
                        d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                        d.ArrivalEvents.pop(0)
                        self.remove(d.LeavingEvents[0])
                        d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                        d.LeavingEvents.pop(0)
                    d.LeavingEvents.insert(0,NewLeavingEvent)
            else: 
                d.RoamingFinished = True

            x=self.elements.pop(0)
            return x


class ListQueueSingleRandomRoaming(ListQueue):

    def removeFirst(self,DriversInSystem,SchedulerType,ShortestPathDictionary,FullDict,MapVertexToNode):
        '''Removes the event that is happening'''
        if len(self.elements) == 0:
            return None
        
        if isinstance(self.elements[0],DriverIntoSystem) :
            d= self.elements[0].Driver
            DriversInSystem.insert(d)
            d.node = d.ArrivalNode
            d.ArrivalEvents.pop(0)

            #Creating a roaming Event
            d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
            E = DriverRoaming(d)
            d.NewRoamingEvent(E)
            self.insertBisection(E)
            if d.LeavingHour <= E.Time:
                self.remove(d.LeavingEvents[0])
                d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                d.LeavingEvents.pop(0)
                NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                d.LeavingHour = E.Time+0.001
                self.insertBisection(NewLeavingEvent)
                while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                    self.remove(d.ArrivalEvents[0])
                    d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                    d.ArrivalEvents.pop(0)
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                d.LeavingEvents.insert(0,NewLeavingEvent)
            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0], DriverLeavesSystem) :
            DriversInSystem.remove(self.elements[0].Driver)
            self.elements[0].Driver.LeavingEvents.pop(0)
            if len(self.elements[0].Driver.LeavingEvents) > 0:
                self.elements[0].Driver.LeavingHour = self.elements[0].Driver.LeavingEvents[0].Time
                self.elements[0].Driver.DesiredLeavingHour  = self.elements[0].Driver.LeavingEvents[0].Time
            self.elements[0].Driver.PseudoEliminatedArrivalEvents = []
            self.elements[0].Driver.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x


        if isinstance(self.elements[0], DriverRoaming):
            ### The roaming has just finished. The driver will go roaming somewhere else ONLY if it has not surpassed his leaving time
            d=self.elements[0].Driver
            ### Before updating the new position of the driver, we will collect statistics:
            type(d).IncreaseDistanceIdle(d,d.TotalDist)
            d.ChangePosition(d.DestinationNode)
            d.RoamingFinished = True
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            x = self.elements.pop(0)
            return x
        
        if isinstance(self.elements[0], PassengerIntoSystem):
            ### Update all the drivers that are roaming:
            ExtraDist = []
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))

            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                x = self.elements.pop(0)
                return x
            elif MinTime > self.elements[0].Passenger.MaxWaitingTime:
                Passenger.IncreaseDriverFarAway()
                x = self.elements.pop(0)
                return x
            else:
                ########                The drive is accepted
                Passenger.IncreaseAcceptedDrives()
                ######## We must update the driven distances:
                if DriverSelected.RoamingFinished == False:
                    DistIdle = DriverSelected.SPDistances[DriverSelected.PartialNodeIndex]
                    type(DriverSelected).IncreaseDistanceIdle(DriverSelected,DistIdle)
                ########                 Creating the pickup event generated by this drive
                NewPickUp = PickUp(self.elements[0],self.elements[0].Passenger,DriverSelected,MinTime)
                self.insertBisection(NewPickUp)
                self.remove(DriverSelected.RoamingEvent)
                ########                 Increase total distance that will be driven
                type(DriverSelected).IncreaseDistanceToPassengers(DriverSelected,MinDist)
                ########                 Remove the driver from the available list
                DriversInSystem.remove(DriverSelected)
                DriverSelected.RoamingFinished = True

                if len(DriverSelected.PseudoEliminatedLeavingEvents) > 0:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents.pop(0)
                    for i in range(0,len(DriverSelected.PseudoEliminatedLeavingEvents)):
                        DriverSelected.LeavingEvents.insert(0,DriverSelected.PseudoEliminatedLeavingEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.LeavingEvents[0])
                    DriverSelected.PseudoEliminatedLeavingEvents = []

                    DriverSelected.LeavingHour = DriverSelected.LeavingEvents[0].Time

                    for i in range(0,len(DriverSelected.PseudoEliminatedArrivalEvents)):
                        DriverSelected.ArrivalEvents.insert(0,DriverSelected.PseudoEliminatedArrivalEvents[len(DriverSelected.PseudoEliminatedLeavingEvents)-1-i])
                        self.insertBisection(DriverSelected.ArrivalEvents[0])

                if DriverSelected.LeavingHour<NewPickUp.Time:
                    self.remove(DriverSelected.LeavingEvents[0])
                    DriverSelected.LeavingEvents[0].ChangeLeavingTime(NewPickUp.Time+0.001)
                    DriverSelected.LeavingHour = NewPickUp.Time+0.001
                    self.insertBisection(DriverSelected.LeavingEvents[0])
                    while (len(DriverSelected.ArrivalEvents) > 0 and DriverSelected.ArrivalEvents[0].Time <= DriverSelected.LeavingHour):
                        self.remove(DriverSelected.ArrivalEvents[0])
                        self.remove(DriverSelected.LeavingEvents[1])
                        DriverSelected.ArrivalEvents.pop(0)
                        DriverSelected.LeavingEvents.pop(1)
                x = self.elements.pop(0)
                return x

        
        if isinstance(self.elements[0],PickUp):
            ###        First we update the new position of the driver
            self.elements[0].Driver.ChangePosition(self.elements[0].Passenger.ArrivalNode)
            ###         Now we compute the shortest path between passenger arrival location and its destination
            MinTime = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][4]
            MinDist = ShortestPathDictionary[(self.elements[0].Driver.node,self.elements[0].Passenger.DestNode)][5]
            ###        We create the dropoff event considering the time it will take to arrive to the destination
            NewDropoffEvent=Dropoff(self.elements[0],self.elements[0].Passenger,self.elements[0].Driver,MinTime)
            self.insertBisection(NewDropoffEvent)
            ###          Increase total distance that will be driven
            type(self.elements[0].Driver).IncreaseDistancePassengers(self.elements[0].Driver,MinDist)
            ####        If the ride exceeds the driver leaving time, a new leaving event for the driver is created
            if NewDropoffEvent.Driver.LeavingHour < NewDropoffEvent.Time:
                self.remove(NewDropoffEvent.Driver.LeavingEvents[0])
                NewDropoffEvent.Driver.LeavingEvents[0].ChangeLeavingTime(NewDropoffEvent.Time+0.001)
                NewDropoffEvent.Driver.LeavingHour = NewDropoffEvent.Time+0.001
                self.insertBisection(NewDropoffEvent.Driver.LeavingEvents[0])
                while(len(NewDropoffEvent.Driver.ArrivalEvents) > 0 and NewDropoffEvent.Driver.ArrivalEvents[0].Time <= NewDropoffEvent.Driver.LeavingEvents[0].Time):
                    self.remove(NewDropoffEvent.Driver.ArrivalEvents[0])
                    self.remove(NewDropoffEvent.Driver.LeavingEvents[1])
                    NewDropoffEvent.Driver.ArrivalEvents.pop(0)
                    NewDropoffEvent.Driver.LeavingEvents.pop(1)

            x = self.elements.pop(0)
            return x

        
        if isinstance(self.elements[0],Dropoff):
            ####          The position of the driver is updated
            d = self.elements[0].Driver
            d.ChangePosition(self.elements[0].Passenger.DestNode)
            ####           The driver is added to the list of available drivers 
            DriversInSystem.insert(d)
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            if d.DesiredLeavingHour > self.elements[0].Time:
                d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
                E = DriverRoaming(d)
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                if d.LeavingHour <= E.Time:
                    self.remove(d.LeavingEvents[0])
                    d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                    d.LeavingEvents.pop(0)
                    NewLeavingEvent = DriverLeavesSystem(d,E.Time+0.001)
                    d.LeavingHour = E.Time+0.001
                    self.insertBisection(NewLeavingEvent)

                    while (len(d.ArrivalEvents) > 0 and d.ArrivalEvents[0].Time <= NewLeavingEvent.Time):
                        self.remove(d.ArrivalEvents[0])
                        d.PseudoEliminatedArrivalEvents.append(d.ArrivalEvents[0])
                        d.ArrivalEvents.pop(0)
                        self.remove(d.LeavingEvents[0])
                        d.PseudoEliminatedLeavingEvents.append(d.LeavingEvents[0])
                        d.LeavingEvents.pop(0)
                    d.LeavingEvents.insert(0,NewLeavingEvent)
            else: 
                d.RoamingFinished = True

            x=self.elements.pop(0)
            return x
            

class GeneralSimulator(object):
    """This class creates the general simulator. The event queue will be a ListQueue object.
    The method doAllEvents will trigger the simulation, as it will only finish once no more events are
    on the queue. Events are removed chronologically, and time is updated to the event time, which allows 
    to know the simulation time at every step"""

    def __init__(self):
        self.events=None
        self.Time=None
    
    def now(self):
        return self.time
    
    def doAllEvents(self, SchedulerType, TAZDict,FullDict,MapVertexToNode):
        """This method executes all events"""
        DriversAvailable = DriversInSystem()
        ExecutedEvents=0
        while self.events.size()>0:
            ExecutedEvents+=1
            e = self.events.removeFirst(DriversAvailable,SchedulerType,TAZDict,FullDict,MapVertexToNode)
            self.Time=e.Time
            if ExecutedEvents%50000 == 0:
                print(str(ExecutedEvents) + " Events have been executed" )
                print(str(len(self.events.elements)) + " events are remaining in the queue")


""" The following two functions were only used in previous versions, when a igraph graph object
    was passed to the queues as input. In fact they were replaced because if one tries to do several
    simulations on the same road network, it is much more efficient to compute all shortest path distances
    once than to do it while executing the algorithm.
    A possible implementation to create the dictionaries is given in CreatingGraph.py, however feel free
    to adjust the code above if you want just to use the Igraph graph and these functions."""

def ComputingShortestPaths(Node,*DriversAvailable,graph):
    """The passenger and the list of drivers available at that moment are passed as argument.
    The graph is passed as argument too. 
    This function computes the shortest path between all drivers and the passenger, and returns
    a list with the respective shortest path distance to the passenger of each driver available"""

    NodesList = []
    for i in range(0,len(DriversAvailable)):
        NodesList.append(DriversAvailable[i].node)
    Paths = graph.get_shortest_paths(Node,NodesList,weights="distances", mode="IN",output="epath")

    Distances = []
    for i in range(0,len(DriversAvailable)):
        dist = 0
        for j in Paths[i]:
            dist+=graph.es[j]["distances"]
        Distances.append(dist)

    return Distances

def Scheduler(Passenger,Node,*Drivers,graph,ExtraDist,DistanceFunction = ComputingShortestPaths):
    """ This function receives as inputs the passenger, a list with all the available drivers, a graph
    and the function used to compute distances.
    If no driver is available, no math driver-passenger can be done. Otherwise, it will compute distances
    between drivers and passengers depending on the metric used, and will select the driver that is closer to the
    passenger.
    The scheduler returns as well the distance between driver and passenger, but the route is not returned."""
    DriversAvailable = list(Drivers)
    ExtraDistance = ExtraDist
    #### Checking first if there are drivers available
    if(len(DriversAvailable) == 0) :
        SelectedDriver="No driver is available"
        MinDist=1000
        return SelectedDriver, MinDist
    
    Dist = DistanceFunction(Node,*DriversAvailable,graph=graph)
    if ExtraDistance != "No Roaming":
        #### Getting shortest paths
        #print("Shortest path distances are : ")
        #print(Dist)
        for i in range(0,len(Dist)):
            Dist[i] += ExtraDistance[i]

    #print("Now they are: ")
    #print(Dist)

    AcceptedRide = 0

    while(AcceptedRide==0):
        if(len(DriversAvailable) == 0) :
            SelectedDriver="No driver is available"
            MinDist=1000
            return SelectedDriver, MinDist
    
        #### Selecting the nearest driver 
        Index = Dist.index(min(Dist))
        SelectedDriver = DriversAvailable[Index]
        if SelectedDriver.Capacity < Passenger.CapacityNeeded or (SelectedDriver.Luxury == False and Passenger.Luxury ==True):
            ### Cannot do the match either because of capacity or luxury constraints
            #print("The ride could not be schedule because of capacity or luxury issues")
            DriversAvailable.pop(Index)
            Dist.pop(Index)

        else:
            MinDist = Dist[Index]
            valueA=np.random.uniform()
            valueB=ProbToAcceptRide(MinDist,SelectedDriver.MaxTimeToAcceptAlways , SelectedDriver.MaxTimeToRejectAlways)
            #### Driver accepts
            if valueA < valueB:
                ##### Ride is accepted
                return SelectedDriver, MinDist

            #### Driver rejects
            else:
                #print("Driver rejected the ride because of distance issues")
                type(DriversAvailable[Index]).IncreaseRejectedDrives(DriversAvailable[Index])
                DriversAvailable.pop(Index)
                Dist.pop(Index)
            