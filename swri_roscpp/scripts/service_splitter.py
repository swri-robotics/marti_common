
import sys

data = sys.argv
#print(data)
#raise ValueError(data)


rf = open(data[1], "rb")
contents = rf.read();
lines = contents.split('---')# split by statement
#split by brackets
print(lines)
rf.close()

#print(data)
#raise ValueError(data)
oi = open(data[2], "wb")
oi.write("marti_common_msgs/ServiceHeader srv_header\n")
oi.write(lines[0])
oi.close()

oi = open(data[3], "wb")
oi.write("marti_common_msgs/ServiceHeader srv_header\n")
oi.write(lines[1])
oi.close()
