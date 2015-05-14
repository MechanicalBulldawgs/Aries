'''
Mean Shift clustering ported from the matlab file MeanShiftCluter.m by Bart Finkston.
The porting was done by Yotam Gingold <yotam (strudel) yotamgingold.com>.

The original code can be found here: http://www.mathworks.com/matlabcentral/fileexchange/10161-mean-shift-clustering
'''

from numpy import *

#function [clustCent,data2cluster,cluster2dataCell] = MeanShiftCluster(dataPts,bandWidth,plotFlag);
def MeanShiftCluster( dataPts, bandWidth, plotFlag = None, nargout = 2 ):
    '''
    perform MeanShift Clustering of data using a flat kernel
    
     ---INPUT---
     dataPts           - input data, (numDim x numPts)
     bandWidth         - is bandwidth parameter (scalar)
     plotFlag          - display output if 2 or 3 D    (logical)
     ---OUTPUT---
     clustCent         - is locations of cluster centers (numDim x numClust)
     data2cluster      - for every data point which cluster it belongs to (numPts)
     cluster2dataCell  - for every cluster which points are in it (numClust)
     
     Bryan Feldman 02/24/06
     MeanShift first appears in
     K. Funkunaga and L.D. Hosteler, "The Estimation of the Gradient of a
     Density Function, with Applications in Pattern Recognition"
     '''
    
    dataPts = asarray( dataPts )
    bandWidth = float( bandWidth )
    
    #*** Check input ****
    if plotFlag is None:
        #plotFlag = True
        plotFlag = False
    else:
        plotFlag = bool( plotFlag )
    
    
    #**** Initialize stuff ***
    numDim, numPts = dataPts.shape
    numClust        = 0
    bandSq          = bandWidth**2
    initPtInds      = arange( numPts )
    maxPos          = dataPts.max(0)                          #biggest size in each dimension
    minPos          = dataPts.min(0)                          #smallest size in each dimension
    boundBox        = maxPos-minPos                        #bounding box size
    sizeSpace       = norm(boundBox)                       #indicator of size of data space
    stopThresh      = 1e-3*bandWidth                       #when mean has converged
    clustCent       = []                                   #center of clust
    beenVisitedFlag = zeros( numPts, dtype = uint8 )              #track if a points been seen already
    numInitPts      = numPts                               #number of points to possibly use as initilization points
    clusterVotes    = [] #zeros( numPts, dtype = uint16 )             #used to resolve conflicts on cluster membership
    
    while numInitPts:
        
        rand = random.rand()
        tempInd         = int(floor( (numInitPts-1e-6)*rand ))        #pick a random seed point
        stInd           = initPtInds[ tempInd ]                  #use this point as start of mean
        myMean          = dataPts[ :, stInd ]                           # intilize mean to this points location
        myMembers       = []                                   # points that will get added to this cluster                          
        thisClusterVotes    = zeros( numPts, dtype = uint16 )         #used to resolve conflicts on cluster membership
        
        while True:     #loop untill convergence
            
            sqDistToAll = (( myMean[:,newaxis] - dataPts )**2).sum(0)    #dist squared from mean to all points still active
            inInds      = where(sqDistToAll < bandSq)               #points within bandWidth
            thisClusterVotes[ inInds ] = thisClusterVotes[ inInds ]+1  #add a vote for all the in points belonging to this cluster
            
            
            myOldMean   = myMean                                   #save the old mean
            myMean      = mean( dataPts[ :, inInds[0] ], 1 )                #compute the new mean
            myMembers.extend( inInds[0] )                       #add any point within bandWidth to the cluster
            beenVisitedFlag[myMembers] = 1                         #mark that these points have been visited
            
            #*** plot stuff ****
            ## NOTE: I have not tested this because matplotlib/pylab are crashing on my machine.
            if plotFlag:
                #import pylab as plt
                import matplotlib.pyplot as plt
                #plt.figure(12345)#,clf,hold on
                if numDim == 2:
                    plt.plot( dataPts[0,:], dataPts[1,:], '.' )
                    plt.plot( dataPts[0,myMembers], dataPts[1,myMembers],'ys')
                    plt.plot( myMean[0], myMean[1], 'go' )
                    plt.plot( myOldMean[0], myOldMean[1], 'rd' )
                    plt.show()
                    #pause
            
            #**** if mean doesn't move much stop this cluster ***
            if norm(myMean-myOldMean) < stopThresh:
                
                #check for merge posibilities
                mergeWith = None
                for cN in xrange( numClust ):
                    distToOther = norm( myMean - clustCent[ cN ] )     #distance from possible new clust max to old clust max
                    if distToOther < bandWidth/2:                    #if its within bandwidth/2 merge new and old
                        mergeWith = cN
                        break
                
                
                if mergeWith is not None:    # something to merge
                    clustCent[ mergeWith ]       = 0.5*( myMean + clustCent[ mergeWith ] )             #record the max as the mean of the two merged (I know biased twoards new ones)
                    #clustMembsCell{mergeWith}    = unique([clustMembsCell{mergeWith} myMembers]);   #record which points inside 
                    clusterVotes[ mergeWith ]    += thisClusterVotes    #add these votes to the merged cluster
                else:    #its a new cluster
                    numClust                    = numClust+1                   #increment clusters
                    clustCent.append( myMean )                       #record the mean  
                    #clustMembsCell{numClust}    = myMembers;                    #store my members
                    clusterVotes.append( thisClusterVotes )
    
                break
        
        initPtInds      = where(beenVisitedFlag == 0)[0]           #we can initialize with any of the points not yet visited
        numInitPts      = len(initPtInds)                   #number of active points in set
    
    data2cluster = asarray( clusterVotes ).argmax(0)                #a point belongs to the cluster with the most votes
    
    #*** If they want the cluster2data cell find it for them
    if nargout > 2:
        '''
        cluster2dataCell = [None] * numClust
        for cN in xrange( numClust ):
            myMembers = where( data2cluster == cN )[0]
            cluster2dataCell[ cN ] = myMembers
        '''
        
        cluster2dataCell = [ where( data2cluster == cN )[0] for cN in xrange( numClust ) ]
    
    if nargout > 2:
        return clustCent, data2cluster, cluster2dataCell
    else:
        return clustCent, data2cluster


def norm( a ):
    '''
    Vector norm, behaves like Matlab's norm when 'a' is a vector.
    '''
    a = asarray( a )
    ## Make sure 'a' is a vector.
    assert prod( a.shape ) == max( a.shape )
    a = a.ravel()
    return sqrt( ( a ** 2 ).sum() )


def test1d():
    print '=== beginning 1D test ==='
    dataPts = asarray([[1],[2],[3],[9],[9],[9],[10]]).T
    #dataPts = [1, 2, 3, 9, 9, 9, 10]
    bandwidth = 2
    print 'data points:', dataPts
    print 'bandwidth:', bandwidth
    clustCent, data2cluster, cluster2data = MeanShiftCluster( dataPts, 2, nargout = 3 )
    print 'cluster centers:', sorted( asarray( clustCent ).squeeze().tolist() )
    print 'data2cluster:', data2cluster
    print 'cluster2data:', cluster2data
    assert len( clustCent ) == 2
    assert sorted( asarray( clustCent ).squeeze().tolist() ) == [ 2., 9.25 ]
    print '=== passed 1D test ==='

def test2d():
    print '=== beginning 2D test ==='
    
    dataPts = asarray( [
        [10,10],[9,9],[10,9],[9,10],
        [10,-10],[9,-9],[10,-9],[9,-10],
        [-10,10],[-9,9],[-10,9],[-9,10],
        [-10,-10],[-9,-9],[-10,-9],[-9,-10]
        ] ).T
    print 'data points:', dataPts
    
    bandwidth = .999
    print 'bandwidth:', bandwidth
    clustCent, data2cluster, cluster2data = MeanShiftCluster( dataPts, bandwidth, nargout = 3)
    numClusters = len( clustCent )
    print 'clustCent:', clustCent
    print 'data2cluster:', data2cluster
    print 'cluster2data:', cluster2data
    assert numClusters == 16
    
    bandwidth = 1.001
    print 'bandwidth:', bandwidth
    clustCent, data2cluster, cluster2data = MeanShiftCluster( dataPts, bandwidth, nargout = 3)
    numClusters = len( clustCent )
    print 'clustCent:', clustCent
    print 'data2cluster:', data2cluster
    print 'cluster2data:', cluster2data
    assert numClusters == 4
    for i in range(0,16,4): assert len( set(data2cluster[i:i+4]) ) == 1
    
    bandwidth = 20.001
    print 'bandwidth:', bandwidth
    clustCent, data2cluster, cluster2data = MeanShiftCluster( dataPts, bandwidth, nargout = 3)
    numClusters = len( clustCent )
    print 'clustCent:', clustCent
    print 'data2cluster:', data2cluster
    print 'cluster2data:', cluster2data
    assert numClusters == 1
    
    print '=== passed 2D test ==='

def main():
    test1d()
    test2d()

if __name__ == '__main__': main()