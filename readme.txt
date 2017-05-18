The key file here is getpolar.m

flight_sample.csv is a 200 point sample from flight.csv, which is useful for quicker testing of the algorithms during development.

The script is expecting a tracklog in a comma seperated file format with the following columns:
1. Row number / ID number
2. Latitude in degrees (D.DDDD..)
3. Longitude in degrees (D.DDDD..)
4. Altitude in meters
5. Date (YYYY/MM/DD)
6. Time (HH:mm:ss)
You can achieve this format by using GPSBabel (https://www.gpsbabel.org/); Convert the tracklog into the Universal csv format and strip the column headings manually afterwards.
