import sys
import os.path
import ntpath

#Function to process the file
def processfile(filename):
    fp = open(filename, "r")
    data = fp.read()
    fp.close()
    
    #Save a backup
    backup = filename + ".org"
    fp = open(backup, "w")
    fp.write(data)
    fp.close()
    
    #Process
    fp = open(filename, "w")
    
    i = 0
    for line in data:
        if(line[-1] == "\0"):
            line = line[:-1]
            i+=1
        fp.write(line)
    fp.close()
        
    print "Changes processed: ", i

#Main
argc = len(sys.argv)
if(argc == 0):
    print "No input files"
    quit()

print "Number of files to process", argc
for x in range(1, argc):    
    filename = str(sys.argv[x])
    if(not os.path.isfile(filename)):
        print 'No such file exist'
        quit()
    else:
        print 'selected blobtree file:', ntpath.basename(filename)
        processfile(filename)
