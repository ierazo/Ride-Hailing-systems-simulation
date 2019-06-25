import igraph 
import numpy as np


##### This function may be used to clean data and duplicate values. It was used for nodes and TAZ.
def RemoveDuplicates(ColumnToEvaluate,ListColumnsToSave):
    CleanColumn = []
    CleanList = []
    for i in range(0,len(ListColumnsToSave)):
        CleanList.append([])

    RepetitionIndices = []
    RepetitionValues =[]
    RepetitionNumber = 0

    for i in range(0,len(ColumnToEvaluate)):
        if ColumnToEvaluate[i] in CleanColumn:
            RepetitionNumber+=1
            RepetitionIndices.append(i)
            RepetitionValues.append(ColumnToEvaluate[i])
        else:
            CleanColumn.append(ColumnToEvaluate[i])
            for j in range(0,len(ListColumnsToSave)):
                CleanList[j].append(ListColumnsToSave[j][i])

    return CleanColumn,CleanList, RepetitionNumber, RepetitionIndices, RepetitionValues

##### This function may be used to create "grid-like" graphs in Igraph.
##### It may be useful lo learn how to create an Igraph graph object
def GridGraph(H_size = 5, V_size=5 , Directed = None):
    """ This function is designed to create and return
    a grid graph. By default it will create a directed graph 
    with 5 rows and 5 columns of nodes. The weights of each node 
    are random. Parameters H_size and V_size must be integer, 
    while parameter Directed can only be None or "yes"  """

    # Verifying that input parameters are correct
    assert type(H_size) == int , "H_size must be integer"
    assert type(V_size) == int , "V_size must be integer"
    assert Directed == None or Directed == "yes" , "Please insert yes or None as value for Directed"

    # Creating graph
    g = igraph.Graph(directed= Directed)  #Option yes allows for it to be directed

    # Adding the correct number of Vertices
    g.add_vertices(H_size*V_size)

    # Creating the edges of the grid with respective weights
    Edge_list=[]
    Distances_list=[]
    Times_list=[]

    for i in range(0,V_size):
        for j in range(0,H_size-1):
            Edge_list.append((i*H_size+j,i*H_size+j+1))
            #Distances_list.append(np.random.randint(1,7)) # This way your custom graphs may have random values
            Distances_list.append(2)
            Times_list.append(5)

    for i in range(0,V_size-1):
        for j in range(0, H_size):
            Edge_list.append((i*H_size+j,(i+1)*H_size+j))   
            #Distances_list.append(np.random.randint(1,7))
            Distances_list.append(2)
            Times_list.append(5)

    g.add_edges(Edge_list)
    g.es["distances"]=Distances_list
    g.es["times"]=Times_list

    # Some settings to plot the graph 
    #layout = g.layout(layout="auto")
    #g.vs["label"]=[i for i in range(0,g.vcount())]
    #g.es["label"]=g.es["distances"]

    # Plotting the graph
    #a=igraph.plot(g,layout=layout)
    #print(a)
    return g


##### This function is used to diver hotspots, however with your real-world data using another algorithm
##### to compute hotspots is recommended
def DiscoveringHotspots(TAZList,ArrivalsLocations):

    a,b,c,d,e = RemoveDuplicates(TAZList,[])
    print(a)
    print(len(a))
    MinNumberHotspots = max([5, int(0.03*len(a))])
    Counter = [0] * len(a)
    for i in ArrivalsLocations:
        for j in range(0,len(a)):
            if i == a[j]:
                Counter[j]+=1

    Counter1=Counter[:]
    a1 = a[:]

    TotalArrivals = sum(Counter)
    Hotspots = []
    ArrivalsInHotspots=[]

    for i in range(0,MinNumberHotspots):
        Index = Counter1.index(max(Counter1))
        Hotspots.append(a1[Index])
        ArrivalsInHotspots.append(Counter1[Index]/TotalArrivals)
        del Counter1[Index]
        del a1[Index]

    return ArrivalsInHotspots , Hotspots


##### This function was used to test the code before doing the study case. 
def CreateArrivalsSpatioTemporally(ListHotspots,ListBlocksOfHours,ListFunctions,ListParameters):
    """
    This function creates arrivals spatio-temporally, given as output an arrival node, with its simulation time in hours.

    ListHotspots is a list that has as elements different list of nodes, each where probabilities of a customer arriving are
    uniform. As an example [[1,3,5],[2],[4] [6,7]] means that there 4 big areas of nodes. If a passenger arrives to one of the
    areas, the probability of arriving to any of the nodes is equal within an area. If a passenger arrives to the area with nodes
    1, 3 and 5, then the probability is equal within those nodes.

    ##ListProbabilities is a list that has lists as elements. Each sublist has as elements the probabilities of arriving to the 
    respective area from ListHotspots at a given BlockOfHours (in Block of hours).

    ListBlocksOfHours is a list that has as values the start and ends of each block of time.

    ListFunctions is a list of lists. Each sublist has as values the functions used to compute the interarrival times between 
    passengers for a given area of nodes. Those are the respective functions for any given Block of hour.

    ListParameters is a list of lists that has as values the parameters needed to use the ListFunctions.

    As an example, CreateArrivalsTemporally( ListHotspots[[1,3,5],[2],[4] [6,7]] , 
    ListProbabilities = [[0.3,0.2],[0.15,0.4],[0.1,0.3],[0.45,0.1]],
    ListBlocksOfHours= [2,6],
    ListFunctions=[[np.random.uniform,np.random.exponential],[np.random.triangular,np.random.triangular],
    [np.random.uniform,np.random.uniform],[np.random.uniform,np.random.uniform]],
    ListParameters = [ [(0,12),10], [(0,10,12),(2,4,8)] , [,] , [,(0,3)] ] )

    is a valid instance, with 4 different areas of nodes and two block of hours, [0,2] and [2,6]. Every passenger has a probability
    to arrive at a given area of node in every block of hours. Within that area the probabilities of each node are assumed equally 
    probable.
    """

    ArrivalsTime = []
    ArrivalsLocation = []

    for i in range(0,len(ListHotspots)):
        TotalTime = 0  
        for j in range(0,len(ListBlocksOfHours)-1):
            while (TotalTime <= ListBlocksOfHours[j+1]):
                InterArrivalTime = ListFunctions[i][j](*ListParameters[i][j])
                TotalTime += InterArrivalTime
                ArrivalsTime.append(TotalTime)
                RandomValue = int(np.random.uniform() * len(ListHotspots[i]))
                ArrivalsLocation.append(ListHotspots[i][RandomValue])

    return ArrivalsTime, ArrivalsLocation


##### Function used to generate the car schedules
def GenerateCarSchedules(NumDrivers,BlocksOfHours,ProbStartingAtBlock,ShiftFunction,Parameters):
    DriversArrivalEvents = []
    DriversLeavingEvents = []
    for i in range(0,NumDrivers):
        EndTimeShift = 0
        DriversArrivalEvents.append([])
        DriversLeavingEvents.append([])
        for j in range(0,len(BlocksOfHours)-1):
            if EndTimeShift <= BlocksOfHours[j]:
                a = np.random.uniform(0,1)
                if a <= ProbStartingAtBlock[j] : 
                    ### The driver will start a shift in the block of time j
                    ArrivalTimeBlock = np.random.uniform(BlocksOfHours[j],BlocksOfHours[j+1])
                    ShiftDuration = ShiftFunction(*Parameters)
                    EndTimeShift = ArrivalTimeBlock + ShiftDuration
                    DriversArrivalEvents[i].append(ArrivalTimeBlock)
                    DriversLeavingEvents[i].append(EndTimeShift)

    return DriversArrivalEvents,DriversLeavingEvents

if __name__ == "__main__":
    import pickle
    import igraph
    import time

    ## Loading Edges Dictionary with info
    pickle_in = open("DictEdges.pickle","rb")
    DictEdges = pickle.load(pickle_in)
    pickle_in.close()

    ## Loading TAZ: nodes dictionary 
    pickle_in = open("3zip2id.pickle","rb")
    DictTAZ = pickle.load(pickle_in)
    DictTAZValues = []
    for i in DictTAZ.values():
        DictTAZValues.append(i)
    pickle_in.close()


    ## Loading Nodes List
    pickle_in = open("ListNodes.pickle","rb")
    ListNodes = pickle.load(pickle_in)
    pickle_in.close()
    
    MapVertexToNode = {}
    pickle_smalldict = open("VertexNumToNode.pickle","wb")
    pickle.dump(MapVertexToNode,pickle_smalldict)
    pickle_smalldict.close()

    ## Creating Big Graph in Igraph
    g = igraph.Graph(directed = "yes")
    number=0
    for i in ListNodes:
        g.add_vertex(name=i)
        MapVertexToNode[number]=i
        number+=1

    for key in DictEdges.keys():
        g.add_edge(source=key[0],target=key[1],times=DictEdges[key][0],distances=DictEdges[key][1])

    pickle_graph = open("FullGraph.pickle","wb")
    igraph.save(g,pickle_graph)
    print(g.es)

    ### LOADING GRAPH
    #pickle_in = open("FullGraph.pickle","rb")
    #g = pickle.load(pickle_in)
    #print(g)
    #pickle_in.close()

    """The following procedures must be used to compute the graphs and save them in a dictionary, but codes
    must be adjusted properly.
    Two dictionaries were needed, as one had more information for the usual origin-destination pairs
    while the other just saved distances and times to speed up calculations.
    If the graph is small enough, the same dictionary could be used as both inputs (because they would be
    not memory issues)"""
    
    # Computing the 10000*969 dict

#    start = time.time()
#    SmallDict = {}
#    for i in range(0,len(ListNodes)):
#        print("Start of evaluation for node "+str(i) + " out of " +str(len(ListNodes)))
#        evaluated=0
#        ShortestPathEdges = g.get_shortest_paths(ListNodes[i],to=DictTAZValues,weights="times", mode="OUT",output="epath")
#        for j in range(0,len(DictTAZValues)):
#            evaluated+=1
#            SumaTimes = 0
#            SumaDist = 0
#            for h in ShortestPathEdges[j]:
#                SumaTimes +=g.es[h]["times"]
#                SumaDist += g.es[h]["distances"]
#            if SumaTimes==0 and ListNodes[i]!=DictTAZValues[j]:
#                InfoDictionary = np.array([-1,-1],dtype=np.float32)
#            else:
#                InfoDictionary = np.array([SumaTimes,SumaDist],dtype=np.float32)
#            SmallDict[(ListNodes[i],DictTAZValues[j])] = InfoDictionary
#        print("The total amount of evaluated routes was " + str(evaluated))

    #pickle_smalldict = open("FullDict.pickle","wb")
    #pickle.dump(SmallDict,pickle_smalldict)
    #pickle_smalldict.close()
    
    #end = time.time()
#    print("Total time needed was " + str(end-start))

    #print("just before opening the Fulldict")
    #pickle_in = open("FullDict.pickle","rb")
    #g = pickle.load(pickle_in)
    #print(g)

    #### 969*969  GRAPH
    #start = time.time()
    #SmallDict = {}
    #for i in range(0,len(DictTAZValues)):
    #    print("Start of evaluation for node "+str(i) + " out of " +str(len(DictTAZValues)))
    #    evaluated=0
    #    ShortestPathEdges = g.get_shortest_paths(DictTAZValues[i],to=DictTAZValues,weights="times", mode="OUT",output="epath")
    #    ShortestPathVertices = g.get_shortest_paths(DictTAZValues[i],to=DictTAZValues,weights="times", mode="OUT",output="vpath")
    #    for j in range(0,len(DictTAZValues)):
    #        evaluated+=1
    #        InfoDictionary = []
    #        SumaTimes = 0
    #        ShortestPathTimes = []
    #        ShortestPathDistances = []
    #        SumaDist = 0
    #        for h in ShortestPathEdges[j]:
    #            SumaTimes +=g.es[h]["times"] 
    #            ShortestPathTimes.append(SumaTimes)
    #            SumaDist += g.es[h]["distances"]
    #            ShortestPathDistances.append(SumaDist)
    #        InfoDictionary.append(ShortestPathTimes)
    #        InfoDictionary.append(ShortestPathDistances)
    #        InfoDictionary.append(ShortestPathEdges[j])
    #        InfoDictionary.append(ShortestPathVertices[j])
    #        if SumaTimes==0 and i!=j:
    #            InfoDictionary.append(-1)
    #            InfoDictionary.append(-1)
    #        else:
    #           InfoDictionary.append(SumaTimes)
    #            InfoDictionary.append(SumaDist)
    #        SmallDict[(DictTAZValues[i],DictTAZValues[j])] = InfoDictionary
    #    print("The total amount of evaluated routes was " + str(evaluated))

    #pickle_smalldict = open("SmallDict.pickle","wb")
    #pickle.dump(SmallDict,pickle_smalldict)
    #pickle_smalldict.close()
    
    #end = time.time()
    #print("Total time needed was " + str(end-start))

    print("done")