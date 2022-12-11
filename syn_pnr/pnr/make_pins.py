startY=25;
startX=25;


pitch=60;
count=0;
# for count in range(0, 13):
# 	print "editPin -snap TRACK -side INSIDE -layer 3 -assign 748 " + str(startY+count*pitch) + "  -pin phase\[" +str(count) + "\]";


for count in range(0, 13):
	print "editPin -snap TRACK -side INSIDE -layer 3 -assign " + str(startX+count*pitch) + " 748 -pin phase\[" +str(count) + "\]";
