class Graph:
     def __init__(self):
          self.children = {}    # dictionnary giving the list of childrens for each node.
          self.q        = []    # configuration associated to each node.
          self.connex   = []    # ID of the connex component the node is belonging to.
          self.nconnex = 0      # number of connex components.
          self.existingConnex = [] # List of existing connex component ID.
          
     def addNode(self, q=None, newConnex=False):
          '''
          Create the memory to store a new edge. Initialize all components to None.
          Create an empty list of children.
          '''
          idx = len(self.children)
          self.children[idx] = []
          self.q.append(q)
          self.connex.append(None)
          if newConnex: self.newConnex(idx)

          return idx

     def addEdge(self,first,second,orientation = 0):
          '''
          Add edge from first to second. Also add edge from second to first if orientation
          is null.
          '''
          assert( first in self.children and second in self.children )
          if orientation>=0: self.children[first].append(second)
          if orientation<=0: self.children[second].append(first)

     def newConnex(self,idx):
          '''
          Create a new connex component for node <idx>
          '''
          self.connex[idx] = self.nconnex
          self.existingConnex.append(self.nconnex)
          self.nconnex += 1

     def renameConnex(self,past,future):
          '''
          Change the index of the all the nodes belonging to a connex component.
          Useful when merging two connex components.
          '''
          try:
               self.existingConnex.remove(past)
               self.connex = [ c if c!=past else future for c in self.connex ]
          except:
               pass

     def connexIndexes(self,connex):
          '''Return the list of all node indexes belonging to connex component <connex>.'''
          return [ i for i,c in enumerate(self.connex) if c == connex ]
