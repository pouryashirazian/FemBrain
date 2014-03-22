#import math
import sys
import os.path
import ntpath
import re

#print 'Number of Args:', len(sys.argv)
if(len(sys.argv) < 2):
    print 'input file missing'
    quit() 
    
filename = str(sys.argv[1])
print 'selected blobtree file:', ntpath.basename(filename)
if(not os.path.isfile(filename)):
    print 'No such file exist'
    quit()
    

verbose = "-v" in sys.argv
#print "verbose", verbose

hashNodeTypeCount = {}
countNodes = countPrims = countOps = 0

i = isec = 0;
curNodeName = "0"
isNode = 0
isOperator = 0
with open(filename) as fp:
    for line in fp:
        line = line.strip(' \0\r\n')
        i = i + 1
        
        ##Find a section        
        if len(line) > 0 and line[0] == '[' and line[-1] == ']':
            secname = line[1:-1]
            isec = isec + 1
            
            #print 'sec', secname
            ##print "Line", i, secname
            if secname.find('BLOBNODE') >= 0:
                #print "node name:", secname
                curNodeName = secname
                countNodes += 1
                isNode = 1
            else:
                isNode = 0
        
        elif isNode and line.find('IsOperator') >= 0:
            fields = re.compile("[=]").split(line)
            isOperator = int(fields[1])
            
            #print 'is op', isOperator, type(isOperator)
            if isOperator == 1:
                countOps += 1
            else:
                countPrims += 1
        elif isNode and isOperator and line.find('OperatorType') >= 0:
            fields = re.compile("[=]").split(line)
            opType = fields[1]
            if(verbose):
                print 'op', opType            
            if(opType in hashNodeTypeCount): 
                hashNodeTypeCount[opType] += 1
            else:
                hashNodeTypeCount[opType] = 1
            
        elif isNode and (not isOperator) and line.find('SkeletonType') >= 0:
            fields = re.compile("[=]").split(line)
            primType = fields[1]
            if(verbose):
                print 'prim', primType
            if(primType in hashNodeTypeCount): 
                hashNodeTypeCount[primType] += 1
            else:
                hashNodeTypeCount[primType] = 1

        elif isNode and (not isOperator) and line.find('PrimitiveType') >= 0:
            fields = re.compile("[=]").split(line)
            primType = fields[1]
            if(verbose):
                print 'prim', primType
            if(primType in hashNodeTypeCount): 
                hashNodeTypeCount[primType] += 1
            else:
                hashNodeTypeCount[primType] = 1
    

if(( countOps + countPrims) == countNodes):
    print 'BlobTree contains ', countOps, ' operators, and ', countPrims, ' primitives, total=', countNodes
    print hashNodeTypeCount

    
    
