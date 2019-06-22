# -*- coding: utf-8 -*-
"""
Created on Fri May 24 10:35:35 2019

@author: erazo
"""
import numpy as np
import igraph
from CreatingGraph import *
import time
from from_to_generator import my_from_to_function
import pickle


TimeLimit = 100
MaxWaitingTime = 7*60

def ShortestPathsInDictionary(g):
    diccionario = {}
    TotalVertices = len(g.vs)
    Step = TotalVertices // 10
    Stops = [0]
    LastValue=0
    while TotalVertices > LastValue+Step:
        LastValue += Step
        Stops.append(LastValue)
    Stops.append(TotalVertices)
    print(Stops)

    start = time.time()

    for i in range(0,len(g.vs)):
        if i in Stops:
            print("Calculo de shortest path " + str(i))
            end=time.time()
            print("Diferencia en tiempo de "+str(end-start))
            start=time.time()


        ShortestPathEdges = g.get_shortest_paths(i,weights="times", mode="OUT",output="epath")
        ShortestPathVertices = g.get_shortest_paths(i,weights="times", mode="OUT",output="vpath")

        for j in range(0,len(g.vs)):
            InfoDictionary = []
            SumaTimes = 0
            ShortestPathTimes = []
            ShortestPathDistances = []
            SumaDist = 0
            for k in ShortestPathEdges[j]:
                SumaTimes+=g.es[k]["times"]
                ShortestPathTimes.append(SumaTimes)
                SumaDist += g.es[k]["distances"]
                ShortestPathDistances.append(SumaDist)

            #En la matriz eje x es salida, eje Y es destino
            InfoDictionary.append(ShortestPathTimes)
            InfoDictionary.append(ShortestPathDistances)
            InfoDictionary.append(ShortestPathEdges[j])
            InfoDictionary.append(ShortestPathVertices[j])
            if SumaTimes==0 and i!=j:
                InfoDictionary.append("cannot")
                InfoDictionary.append('cannot')
            else:
                InfoDictionary.append(SumaTimes)
                InfoDictionary.append(SumaDist)
            diccionario[(i,j)] = InfoDictionary
    return diccionario

def SchedulerWithDictionary(Passenger,Node,*Drivers,ExtraDist,dictionary):
    #print("Entered")
    DriversAvailable = list(Drivers)
    ExtraDistance = ExtraDist
    #### Checking first if there are drivers available
    if(len(DriversAvailable) == 0) :
        SelectedDriver="No driver is available"
        MinTime=1000
        MinDist = "No driver"
        return SelectedDriver, MinTime, MinDist

    #### As there are some drivers available, we need to get their shortest path times into a list
    Time = []
    DriversThatCanGo = []
    #print(DriversAvailable)
    for i in range(0,len(DriversAvailable)):
        #print("the driver is: ")
        #print(DriversAvailable[i])
        #print("its node is: ")
        #print(DriversAvailable[i].node)
        #print(dictionary[(DriversAvailable[i].node,Node)])
        if dictionary[(DriversAvailable[i].node,Node)][0] >= 0:
            Time.append(dictionary[(DriversAvailable[i].node,Node)][0])
            DriversThatCanGo.append(DriversAvailable[i])

    if ExtraDistance != "No Roaming":
        #### Getting shortest paths
        #print("Shortest path distances are : ")
        #print(Dist)
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
            #print("The ride could not be schedule because of capacity or luxury issues")
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
    """This class is used to characterize when a driver enters the system. All of them arrive at time 0."""
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
    
class PassengerLeavesSystem(Event):
    """This class is implemented but not used yet. It may be deleted later, as the dropoff events are
    the moment when passengers leaves the system"""
    def __init__(self,Time,Passenger):
        self.Time=Time
        self.Passenger = Passenger
        
    def __str__(self):
        result = "Passenger " + str(self.Passenger.Number) + " Leaves at time " + str(self.Time)
        return result

class DriversInSystem(object):
    """This class is just a list with all the available drivers at any moment. It gets updated when 
    assignments are made, or when dropoff events are concluded. 
    
    It is needed to test the efficiency of this method, changing it to another idea like conditionals, or 
    inserting it at the latest position instead of the first."""
    def __init__(self):
        self.elements=list()
    def insert(self,Driver):
        self.elements.insert(0,Driver)
        #self.elements.append(Driver)
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
        #if self.ArrivalNode == self.DestNode:
        #    self.DestNode += 1
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
    """This class represent the drivers. They are initialized randomly in a node.
    They are given as input to a entering event (which needs information about which driver enters the system)
    which means they will enter the system depending on the structure defined in the entering event. They have
    a leaving event too, with a predefined but modifiable leaving time
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
        #type(self).__NumberOfRoamings += 1

    def EstablishLuxury(self):
        a = np.random.uniform(0,1)
        if a < 0.9:
            self.Luxury = False
        else:
            self.Luxury = True


    def EstablishCapacity(self):
        """ Establishing the capacity allowed for each car """
        a = np.random.uniform()
        if a < (1-0.164):   ### Just a normal car
            self.Capacity = np.random.randint(3,5)
        else:
            self.Capacity = np.random.randint(5,7)

    def EstablishSchedule(self,ArrivalEvents,LeavingEvents):
        self.ArrivalEvents = ArrivalEvents  
        self.LeavingEvents = LeavingEvents
        self.LeavingHour = LeavingEvents[0].Time
        self.DesiredLeavingHour = self.LeavingHour  ### This condition is only used in roaming procedures

    def EstablishArrivalLocation(self,ListTAZ):
        value = np.random.randint(0,len(ListTAZ))
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
    """This class represent the drivers. They are initialized randomly in a node.
    They are given as input to a entering event (which needs information about which driver enters the system)
    which means they will enter the system depending on the structure defined in the entering event. They have
    a leaving event too, with a predefined but modifiable leaving time
    These drivers only move when they need to go pick up a passenger and dropoff them. They change their node
    to the destination node and then wait there (they do not hunt for passengers)"""
    
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
        if MinimumVal < 2:
            ###### At least 1 hotspot has less than 2 cars routed there, so we will restrict the analysis to those hotspots
            for i in range(0,len(type(self).RoutedCarsToHotspots)):
                if type(self).RoutedCarsToHotspots[i]==MinimumVal:
                    PossibleHotspots.append(type(self).Hotspots[i])
                    Priorities.append(type(self).PrioritiesHotspots[i])
        else: 
            ####### otherwise all hotspots are evaluated
            PossibleHotspots = type(self).Hotspots[:]
            Priorities = type(self).PrioritiesHotspots[:]
        #print("El vehiculo se encuentra en el nodo " + str(self.node) + " ,los hotspots a considerar para el ruteo son: ")
        #print(PossibleHotspots)
        #print("Con prioridades respectivas de: ")
        #print(Priorities)
        ShortestPathEdges = []
        Distances = []
        for j in PossibleHotspots:
            Distances.append(dictionary[(self.node,j)][4])
        
        Prioridad = []
        for i in range(0,len(Distances)):
            Prioridad.append(Priorities[i]/(Distances[i]+1))
        #print("Ajustando por distancia las prioridades de lso hotspots son: ")
        #print(Prioridad)
        #Getting the highest value of total priority
        Index = Prioridad.index(max(Prioridad))
        self.DestinationNode = PossibleHotspots[Index]
        #result = "El hotspot con mayor prioridad total es " + str(self.DestinationNode)
        #result = " previo al ruteo el contador de autos ruteados es "
        #print(result)
        #print(Cars.RoutedCarsToHotspots)
        ShortestPathEdges = dictionary[(self.node,self.DestinationNode)][2]
        ShortestPathVertices = dictionary[(self.node,self.DestinationNode)][3]
        ShortestPathTimes = dictionary[(self.node,self.DestinationNode)][0]
        ShortestPathDistances = dictionary[(self.node,self.DestinationNode)][1]
        ShortestPathTotalTimes=dictionary[(self.node,self.DestinationNode)][4]
        TotalDist = dictionary[(self.node,self.DestinationNode)][5]

        Index = type(self).Hotspots.index(self.DestinationNode)
        type(self).RoutedCarsToHotspots[Index]+=1
        #print("Ahora el numero de autos ruteados es: ")
        #print(Cars.RoutedCarsToHotspots)
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
        Distances = []
        for j in (type(self).Hotspots):
            Distances.append(dictionary[self.node,j][4])

        Index = Distances.index(min(Distances))
        self.DestinationNode = type(self).Hotspots[Index]
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
        if self.StartRoamingTime < 7*24*3600:
            a,b =my_from_to_function(self.StartRoamingTime, day_hours_intervals, days_intervals, hourly_blocks,
                    threeZip2osmid, summary_dictionary_all, final_dict,start5zip=Dict3ZipTo5Zip[DictNodeTo3Zip[self.node]],status='idle', verbose=False)
            self.DestinationNode = a[1][0]
        else:
            a = np.random.randint(0,self.NodesInGraph)
            self.DestinationNode = self.ListTAZ[a]

        while self.node == self.DestinationNode:
            if self.StartRoamingTime < 7*24*3600:
                a,b =my_from_to_function(self.StartRoamingTime, day_hours_intervals, days_intervals, hourly_blocks,
                        threeZip2osmid, summary_dictionary_all, final_dict,start5zip=Dict3ZipTo5Zip[DictNodeTo3Zip[self.node]],status='idle', verbose=False)
                self.DestinationNode = a[1][0]
            else:
                a = np.random.randint(0,self.NodesInGraph)
                self.DestinationNode = self.ListTAZ[a]

        ShortestPathEdges = dictionary[(self.node,self.DestinationNode)][2]
        ShortestPathVertices = dictionary[(self.node,self.DestinationNode)][3]
        ShortestPathTimes = dictionary[(self.node,self.DestinationNode)][0]
        ShortestPathDistances = dictionary[(self.node,self.DestinationNode)][1]
        ShortestPathTotalTimes=dictionary[(self.node,self.DestinationNode)][4]
        TotalDist = dictionary[(self.node,self.DestinationNode)][5]
        
        return ShortestPathTimes, ShortestPathDistances, ShortestPathEdges, ShortestPathVertices, ShortestPathTotalTimes, TotalDist

class CarsContinuousRandomRoaming(CarsSingleRandomRoaming):

    pass


class ListQueue(object):
    """This class represents the simulation queue. Events are inserted depending on their execution time.
    Events are removed once the simulation executes them (chronologically). 
    Depending on the type of event, another may be created and scheduler. As an example, a pick-up event will 
    trigger the creation of a dropoff event.
    """
    elements = list()

    def insert(self,x):
        '''Inserts event x in the position i'''
        i=0
        ###### It starts looking at each position and comparing with the overloaded operator <
        while i < len(self.elements) and (self.elements[i] <= x):
            i += 1
        self.elements.insert(i,x)
        return

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
        ####   First it searches the event so it can eliminate it from the queue
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
            DriversInSystem.insert(self.elements[0].Driver)
            self.elements[0].Driver.node = self.elements[0].Driver.ArrivalNode
            self.elements[0].Driver.ArrivalEvents.pop(0)
            x = self.elements.pop(0)
            return x
        
        if isinstance(self.elements[0], DriverLeavesSystem) :
            DriversInSystem.remove(self.elements[0].Driver)
            self.elements[0].Driver.LeavingEvents.pop(0)
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
                #print("No driver is available so the travel could not be scheduled")
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
            
            
        if isinstance(self.elements[0], PassengerLeavesSystem) :
            x = self.elements.pop(0)
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
            #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            #print("Passenger arrives at time: " + str(self.elements[0].Time))
            ExtraDist = []
            #print("There are " + str(len(DriversInSystem.elements)))
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        #print("DeltaTime is: " + str(DeltaTime))
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        #print("PartialNode is: " + str(i.SPVertices[0][check]))
                        #print("Extra distance to node " + str(i.SPVertices[0][check+1]) + "is " + str(i.SPDistances[check]-DeltaTime))
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))

            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                #print("No driver is available so the travel could not be scheduled")
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
            #print("Distance to dropoff is: "+ str(MinDist[0]))
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

            #print(str(self.elements[0])+ " time to go to the destination is " +str(MinTime)+" dist to go is " + str(MinDist))
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
                #print("a new roaming event was created, going to " + str(d.DestinationNode))
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            
            
        if isinstance(self.elements[0], PassengerLeavesSystem) :
            x = self.elements.pop(0)
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
            #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            #print("Passenger arrives at time: " + str(self.elements[0].Time))
            ExtraDist = []
            #print("There are " + str(len(DriversInSystem.elements)))
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        #print("DeltaTime is: " + str(DeltaTime))
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        #print("PartialNode is: " + str(i.SPVertices[0][check]))
                        #print("Extra distance to node " + str(i.SPVertices[0][check+1]) + "is " + str(i.SPDistances[check]-DeltaTime))
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))
            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                #print("No driver is available so the travel could not be scheduled")
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
            #print("Distance to dropoff is: "+ str(MinDist[0]))
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

            #print(str(self.elements[0])+ " time to go to the destination is " +str(MinTime)+" dist to go is " + str(MinDist))
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
                #print("a new roaming event was created, going to " + str(d.DestinationNode))
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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

            
        if isinstance(self.elements[0], PassengerLeavesSystem) :
            x = self.elements.pop(0)
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
            #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            #print("Passenger arrives at time: " + str(self.elements[0].Time))
            ExtraDist = []
            #print("There are " + str(len(DriversInSystem.elements)))
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        #print("DeltaTime is: " + str(DeltaTime))
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        #print("PartialNode is: " + str(i.SPVertices[0][check]))
                        #print("Extra distance to node " + str(i.SPVertices[0][check+1]) + "is " + str(i.SPDistances[check]-DeltaTime))
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))

            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)
            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                #print("No driver is available so the travel could not be scheduled")
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
            #print("Distance to dropoff is: "+ str(MinDist[0]))
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

            #print(str(self.elements[0])+ " time to go to the destination is " +str(MinTime)+" dist to go is " + str(MinDist))
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
                #print("a new roaming event was created, going to " + str(d.DestinationNode))
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            
        if isinstance(self.elements[0], PassengerLeavesSystem) :
            x = self.elements.pop(0)
            return x

class ListQueueContinuousRandomRoaming(ListQueue):

    def removeFirst(self,DriversInSystem,SchedulerType,ShortestPathDictionary,FullDict,MapVertexToNode):

        #print(self.elements[0])

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
            #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            d.PseudoEliminatedArrivalEvents = []
            d.PseudoEliminatedLeavingEvents = []
            if (d.DesiredLeavingHour>self.elements[0].Time):
                d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
                E = DriverRoaming(d)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
                #print(str(self.elements[0]))
                #print(str(E))
                #print("a new roaming event was created, going to " + str(d.DestinationNode))
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
        
        if isinstance(self.elements[0], PassengerIntoSystem):
            ### Update all the drivers that are roaming:
            #print("Passenger arrives at time: " + str(self.elements[0].Time))
            ExtraDist = []
            #print("There are " + str(len(DriversInSystem.elements)))
            if len(DriversInSystem.elements)>0:
                for i in DriversInSystem.elements:
                    if i.RoamingFinished == True:
                        ExtraDist.append(0)
                    else:
                        DeltaTime = self.elements[0].Time - i.StartRoamingTime
                        #print("DeltaTime is: " + str(DeltaTime))
                        check = i.PartialNodeIndex
                        while i.SPTimes[check] <= DeltaTime and check < len(i.SPTimes):
                            check+=1
                        i.PartialNodeIndex = check
                        #print("node is "+ str(i.node))
                        #print("SPVertices is : " )
                        #print(i.SPVertices)
                        i.ChangePosition(MapVertexToNode[i.SPVertices[check+1]])
                        #print("PartialNode is: " + str(i.SPVertices[0][check]))
                        #print("Extra distance to node " + str(i.SPVertices[0][check+1]) + "is " + str(i.SPDistances[check]-DeltaTime))
                        ExtraDist.append((i.SPTimes[check]-DeltaTime))

            DriverSelected , MinTime, MinDist = SchedulerType(self.elements[0].Passenger,self.elements[0].Passenger.ArrivalNode,*DriversInSystem.elements,ExtraDist=ExtraDist,dictionary = FullDict)

            if DriverSelected == "No driver is available":
                #######                 The drive is rejected
                Passenger.IncreaseNonDriverAvailable()
                #print("No driver is available so the travel could not be scheduled")
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
                ######## We must update the driven distances:
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
                ########       Roaming finishing event dismissed, we are getting all events that have not passed


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
            #print("Distance to dropoff is: "+ str(MinDist[0]))
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

            #print(str(self.elements[0])+ " time to go to the destination is " +str(MinTime)+" dist to go is " + str(MinDist))
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
            if (d.DesiredLeavingHour>self.elements[0].Time):
                d.CreateRoaming(ShortestPathDictionary,self.elements[0].Time)
                E = DriverRoaming(d)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
                #print(str(self.elements[0]))
                #print(str(E))
                #print("a new roaming event was created, going to " + str(d.DestinationNode))
                d.NewRoamingEvent(E)
                self.insertBisection(E)
                #print("a new roaming event was created for driver " + str(d.Number) + " finishing at time " +str(E.Time) )
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
            
            
        if isinstance(self.elements[0], PassengerLeavesSystem) :
            x = self.elements.pop(0)
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
        #print(DriversAvailable.elements)
        ExecutedEvents=0
        while self.events.size()>0:
            ExecutedEvents+=1
            e = self.events.removeFirst(DriversAvailable,SchedulerType,TAZDict,FullDict,MapVertexToNode)
            self.Time=e.Time
            if ExecutedEvents%50000 == 0:
                print(str(ExecutedEvents) + " Events have been executed" )
                print(str(len(self.events.elements)) + " events are remaining in the queue")
            #print(Cars.RoutedCarsToHotspots)
            #result = "Executed: " + str(e)
            #print(result)
            #print("There are " + str(len(DriversAvailable.elements)) + " drivers" + "\n")
            #if isinstance(e,Dropoff):
            #    pas = e.Passenger
            #    del(e)
            #    del(pas)
            #else: 
            #    del(e)

            
            
    
if __name__=='__main__':
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



    Proba = []
    newarray1 = [0.002,0.002,0.002,0.007,0.011,0.03,0.055,0.028]
    newarray2 = [0.03,0.03,0.03,0.025,0.025,0.03,0.04,0.05]
    newarray3 = [0.085,0.095,0.04,0.05,0.04,0.03,0.015,0.012]
    newarray4 = [0.07,0.105,0.07,0.07,0.055,0.055,0.05,0.018]

    for i in range(0,4):
        Proba.extend(newarray1)
        Proba.extend(newarray2)
        Proba.extend(newarray3)

    Proba.extend(newarray1)
    Proba.extend(newarray2)
    Proba.extend(newarray4)

    for i in range(0,2):
        Proba.extend(newarray1)
        Proba.extend(newarray2)
        Proba.extend(newarray3)

    for i in range(0,len(Proba)):
        Proba[i]=Proba[i]/1.6

    ## TimeLimit for the simulation is established in the classes.py file


    Cars = [CarsNoRoaming,CarsSingleRandomRoaming,CarsNearestHotspot,CarsSpecificHotspot,CarsContinuousRandomRoaming]

    Queue = [ListQueueNoRoaming,ListQueueSingleRandomRoaming, ListQueueNearestHotspot, ListQueueSpecificHotspot,ListQueueContinuousRandomRoaming]

    #### MAPPING TAZ NODES
    start = time.time()
    pickle_in = open("3zip2id.pickle","rb")   #abrir file
    TAZ = pickle.load(pickle_in)              #traspasar a diccionario actual
    TAZList = []
    #DictNodeTo3zip = {}
    for key,i in TAZ.items():
        if i in TAZList:
            a=0
        else:
            TAZList.append(str(i))
            #DictNodeTo3zip[i] = key
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the TAZList was: " + str(end-start))



    #### 969 * 969 DICT

    start = time.time()
    pickle_in = open("SmallDict.pickle","rb") #abrir file
    TAZDict = pickle.load(pickle_in)          #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the TAZDict dictionary was: " + str(end-start))


    start = time.time()
    pickle_in = open("VertexNumToNode.pickle","rb") #abrir file
    MapVertexToNode = pickle.load(pickle_in)          #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the MapVertex dictionary was: " + str(end-start))

    #### 11372 * 969 DICT

    start = time.time()
    pickle_in = open("FullDict.pickle","rb")  #abrir file
    FullDict = pickle.load(pickle_in)         #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the FullDict dictionary was: " + str(end-start))
    print(len(FullDict))


    #### MAPPING TAZ NODES
    start = time.time()
    pickle_in = open("3zip2id.pickle","rb")   #abrir file
    TAZ = pickle.load(pickle_in)              #traspasar a diccionario actual
    TAZList = []
    for i in TAZ.values():
        if i in TAZList:
            a=0
        else:
            TAZList.append(str(i))
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the TAZList was: " + str(end-start))
    
    #### ARRIVALS
    start = time.time()
    #pickle_in = open("SmallInstances10repVar1Cor0.pickle","rb")
    pickle_in = open("Real10ReplicasSFCorr0.pickle","rb")  #abrir file
    ArrivalsTimes = pickle.load(pickle_in)         #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file
    end = time.time()
    print("Total time needed to get the ArrivalTimes of the small instances was: " + str(end-start))

    #### CARGANDO DATOS PARA SIMULAR LLEGADAS ESPACIALMENTE
    with open('C:/Users/erazo/Desktop/Universidad/6to/Memoria Uber/Codigos python/Codigos_Simulacion/3zip2id.pickle', "rb") as input_file:
        threeZip2osmid = pickle.load(input_file)
    #print(len(threeZip2osmid))
    with open('C:/Users/erazo/Desktop/Universidad/6to/Memoria Uber/Codigos python/Codigos_Simulacion/demand_dictionary.pickle', "rb") as input_file:
        summary_dictionary_all = pickle.load(input_file)
    with open('C:/Users/erazo/Desktop/Universidad/6to/Memoria Uber/Codigos python/Codigos_Simulacion/from_too_dict.pickle', "rb") as input_file:
        final_dict = pickle.load(input_file)

    
    pickle_in = open("Dict3Zipto5Zip.pickle","rb") #abrir file
    Dict3ZipTo5Zip = pickle.load(pickle_in)          #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file


    pickle_in = open("DictNodeTo3zip.pickle","rb") #abrir file
    DictNodeTo3Zip = pickle.load(pickle_in)          #traspasar a diccionario actual
    pickle_in.close()                         #cerrar file

    #Corriendo simulacion

    ################ DESDE AQUI HACE FALTA ARREGLAR INPUTS

    for REPLICAS in range(1,10):
        # Number of cars and passengers of the simulation
        #NumPassengers = 50
        #NumCars = 7500

        # Iniciando listas para recibir resultados
        PassengerNumber = [ [] , [] , [] , [] , []  ] 
        AcceptedDrives = [ [] , [] , [] , [] , []  ]  
        NonDriverAvailable = [ [] , [] , [] , [] , []  ]
        DriverFarAway = [ [] , [] , [] , [] , []  ]
        CarsNumber= [ [] , [] , [] , [] , []  ]
        DriverRejecting = [ [] , [] , [] , [] , []  ]
        DistWithPassengers = [ [] , [] , [] , [] , []  ]
        DistanceIdle = [ [] , [] , [] , [] , []  ]
        DistToPas = [ [] , [] , [] , [] , []  ]
        NumRoamings = [ [] , [] , [] , [] , []  ]

        ### AQUI INICIA SF
        start = time.time()
        np.random.seed(REPLICAS)

        ####### PASSENGERS
        ### SE CARGAN LAS LLEGADAS DE LA REPLICA
        Arriv=ArrivalsTimes[REPLICAS]*3600
        #print("length of arrivals is: "+str(len(Arriv)))

        hours_intervals = [(((i-1)*hour_in_sec,i*hour_in_sec), tag) if i==1 else
                    (((i-1)*hour_in_sec+1,i*hour_in_sec), tag) for i, tag in hours_list]
        days_intervals = [((i*day_in_sec,(i+1)*day_in_sec), tag) if i==0 else 
                    ((i*day_in_sec+1,(i+1)*day_in_sec), tag) for i, tag in days_list]
        day_hours_intervals = {j:[((k[0][0]+i[0],k[0][1]+i[0]),k[1]) for k in hours_intervals] for i, j in days_intervals}

        id2id = [None]*len(Arriv[0]) # it is easier to index than to append
        five2five = [None]*len(Arriv[0])
        for i, tm in enumerate(list(Arriv[0])):
            a1, b1 = my_from_to_function(tm, day_hours_intervals, days_intervals, hourly_blocks,
                    threeZip2osmid, summary_dictionary_all, final_dict, verbose=False)

            id2id[i] = a1
            five2five[i] = b1
        
        print("Arrivals are done")
        ## id2id es el diccionario con las llegadas y destinos de pasajeros

        ArrivalsList = [None]* len(id2id)
        for i in range(0,len(id2id)):
            ArrivalsList[i]=id2id[i][0][0]
    
        ArrivalsInHotspots,Hotspots = DiscoveringHotspots(TAZList,ArrivalsList)
        #print("Hotspots discovered")
        #print(Hotspots)
        #print(ArrivalsInHotspots)

        CarsSpecificHotspot.Hotspots = Hotspots
        CarsSpecificHotspot.PrioritiesHotspots = ArrivalsInHotspots
        CarsSpecificHotspot.RoutedCarsToHotspots = []

        for i in range(0,len(Hotspots)):
            CarsSpecificHotspot.RoutedCarsToHotspots.append(0)
        CarsNearestHotspot.Hotspots = Hotspots

        NumCars = 16200
        BlockOfHours = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,
        24*1+1,24*1+2,24*1+3,24*1+4,24*1+5,24*1+6,24*1+7,24*1+8,24*1+9,24*1+10,24*1+11,24*1+12,24*1+13,24*1+14,24*1+15,24*1+16,24*1+17,24*1+18,24*1+19,24*1+20,24*1+21,24*1+22,24*1+23,24*1+24,
        24*2+1,24*2+2,24*2+3,24*2+4,24*2+5,24*2+6,24*2+7,24*2+8,24*2+9,24*2+10,24*2+11,24*2+12,24*2+13,24*2+14,24*2+15,24*2+16,24*2+17,24*2+18,24*2+19,24*2+20,24*2+21,24*2+22,24*2+23,24*2+24,
        24*3+1,24*3+2,24*3+3,24*3+4,24*3+5,24*3+6,24*3+7,24*3+8,24*3+9,24*3+10,24*3+11,24*3+12,24*3+13,24*3+14,24*3+15,24*3+16,24*3+17,24*3+18,24*3+19,24*3+20,24*3+21,24*3+22,24*3+23,24*3+24,
        24*4+1,24*4+2,24*4+3,24*4+4,24*4+5,24*4+6,24*4+7,24*4+8,24*4+9,24*4+10,24*4+11,24*4+12,24*4+13,24*4+14,24*4+15,24*4+16,24*4+17,24*4+18,24*4+19,24*4+20,24*4+21,24*4+22,24*4+23,24*4+24,
        24*5+1,24*5+2,24*5+3,24*5+4,24*5+5,24*5+6,24*5+7,24*5+8,24*5+9,24*5+10,24*5+11,24*5+12,24*5+13,24*5+14,24*5+15,24*5+16,24*5+17,24*5+18,24*5+19,24*5+20,24*5+21,24*5+22,24*5+23,24*5+24,
        24*6+1,24*6+2,24*6+3,24*6+4,24*6+5,24*6+6,24*6+7,24*6+8,24*6+9,24*6+10,24*6+11,24*6+12,24*6+13,24*6+14,24*6+15,24*6+16,24*6+17,24*6+18,24*6+19,24*6+20,24*6+21,24*6+22,24*6+23,24*6+24]
        ProbStartingAtBlock = Proba

        for i in range(0,len(BlockOfHours)):
            BlockOfHours[i]=3600*BlockOfHours[i]
    
        DriversArrivalEvents,DriversLeavingEvents = GenerateCarSchedules(NumCars,
        BlocksOfHours=BlockOfHours,ProbStartingAtBlock=ProbStartingAtBlock
        , ShiftFunction=np.random.triangular , Parameters=[1*3600,3*3600,6*3600])

        end = time.time()

        print("The setup of replica " + str(REPLICAS) + " took " + str(end-start) )
        #### Right now id2id has the origin-destinations and ArrivalTimes the arrival times of passengers

        #NumHotspots= max(5,int(0.02*hsize*vsize))
        #ProbHotspot = 1

        ### Creating the supposed hotspots
        #TheoricalHotspots = CreateHotspots(vsize,hsize,Hotspot_H_size=5,Hotspot_V_size=5,Number_of_hotspots=4)

        #BlocksOfHours  = [0,6,9,12,15,18,20,30]
        ### Creating the arrivals
        #expo = np.random.exponential
        #func = [expo,expo,expo,expo,expo,expo,expo]

        #ArrivalsTime,ArrivalsLocation = CreateArrivalsSpatioTemporally(TheoricalHotspots,BlocksOfHours,
        #[func, func , func, func,func], 
        #[ [[1/.5],[1/.7],[1/.2],[1/.4],[1/.2],[1/2.5],[1/2.5]] , [[1/.5],[1/.7],[1/.2],[1/.4],[1/.2],[1/2.5],[1/2.5]]  ,
        #[[1/.5],[1/.7],[1/.2],[1/.4],[1/.2],[1/2.5],[1/2.5]]  , [[1/.5],[1/.7],[1/.2],[1/.4],[1/.2],[1/2.5],[1/2.5]] , 
        #[[1/.5],[1/.7],[1/.2],[1/.4],[1/.2],[1/2.5],[1/2.5]]  ] )

        #print("Arrival Times are: ")
        #print(ArrivalsTime)
        #print("Arrival Locations are: ")
        #print(ArrivalsLocation)

        ### Determining real hotspot locations from the arrival data
        #Center, Counter, Hotspots, ArrivalsInHotspots = ObtainingHotspots(vsize,hsize,ArrivalsLocation)
        ### Using results from the function as the parameters for the simulation model
        #CarsSpecificHotspot.Hotspots = Hotspots
        #CarsSpecificHotspot.PrioritiesHotspots = ArrivalsInHotspots
        #CarsSpecificHotspot.RoutedCarsToHotspots = []

        #for i in range(0,len(Hotspots)):
        #    CarsSpecificHotspot.RoutedCarsToHotspots.append(0)
        #CarsNearestHotspot.Hotspots = Hotspots

        #DriversArrivalEvents,DriversLeavingEvents = GenerateCarSchedules(NumCars,
        #BlocksOfHours=[0,6,9,12,15,18,20,30],ProbStartingAtBlock=[0.2,0.6,0.4,0.3,0.35,0.45,0.75]
        #, ShiftFunction=np.random.triangular , Parameters=[1,4,8])

        for j in range(0,4):

            start= time.time()

            np.random.seed(REPLICAS)
            hours_intervals = [(((i-1)*hour_in_sec,i*hour_in_sec), tag) if i==1 else
                    (((i-1)*hour_in_sec+1,i*hour_in_sec), tag) for i, tag in hours_list]
            days_intervals = [((i*day_in_sec,(i+1)*day_in_sec), tag) if i==0 else 
                    ((i*day_in_sec+1,(i+1)*day_in_sec), tag) for i, tag in days_list]
            day_hours_intervals = {j:[((k[0][0]+i[0],k[0][1]+i[0]),k[1]) for k in hours_intervals] for i, j in days_intervals}

            ActualQueue = Queue[j]()
            print("\n \n")
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

            print("Creating drivers took "+ str(end-start))
            print("The size of the queue created is: "+str(len(ActualQueue.elements)))
            print("\n ")
            start= time.time()
            print("Starting to run the simulation number " + str(j+1) + " replica number " + str(REPLICAS+1))

            Sim=GeneralSimulator()
            Sim.events=ActualQueue
            #print(Sim.events)
            Sim.doAllEvents(SchedulerWithDictionary,TAZDict,FullDict,MapVertexToNode)

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

            Cars[j].ClearDriversInformation(x)
            Passenger.ClearPassengersInformation()
            if j == 4:
                print(Cars[j].RoutedCarsToHotspots)

            end = time.time()
            del(ActualQueue)
            print("One replica of scenario " + str(j+1) + " took " + str(end-start) )


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
        #NameOfSavedPickle = "TryingNewCode" + str(REPLICAS)
        pickle_disconnected = open(NameOfSavedPickle,"wb")
        pickle.dump(Results,pickle_disconnected)
        pickle_disconnected.close()



    print("Debugging done")
