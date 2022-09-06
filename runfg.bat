C:

cd C:\Program Files\FlightGear 2020.3

SET FG_ROOT=C:\Program Files\FlightGear 2020.3\data

START .\\bin\fgfs.exe --fdm=null --native-fdm=socket,in,30,localhost,5502,udp --native-fdm=socket,out,30,localhost,5501,udp --aircraft=SR22T --in-air --airport=KBOS --runway=14 --altitude=2000 --heading=113 --offset-distance=0 --offset-azimuth=0