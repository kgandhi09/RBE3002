#Problem 5

import csv
import math

csv_path = 'HW3/RBE3002_D20_City_Data.csv'

readList = []

def haversine(city1, city2, file):

    R = 6371000   # Radius of Earth

#Reading from file and converting to list of strings
    for row in file:
        readList.append(row)

#Converting longitudes and lattitude values to integers
    for i in range(len(readList)):
        if i > 0:
            readList[i][1] = float(readList[i][1])
            readList[i][2] = float(readList[i][2])
            readList[i][4] = float(readList[i][4])
            readList[i][5] = float(readList[i][5])

    #Greenfield coordinates
    lat1 = 0
    long1 = 0
    lat2 = readList[1][4]
    long2 = readList[1][5]

    #Getting the coordinates of cities
    for row in readList:
        if city1 == row[0]:
            lat1 = row[1]
            long1 = row[2]

    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(long2 - long1)

    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    meters = R * c  # output distance in meters
    km = meters / 1000.0  # output distance in kilometers

    km = round(km, 2)
    miles = km * 0.621371
    miles = round(miles, 2)

    print city1 + ' is ' + str(km) + ' km (' + str(miles) + ' miles) from ' + city2

if __name__ == "__main__":
    file = csv.reader(open(csv_path, 'r'))
    city_list = ['Boston', 'Fall River', 'Framingham', 'Greenfield', 'Hartford', 'Leominster', 'Lowell', 'Nashua', 'North Adams', 'Northampton',
                 'Pittsfield', 'Providence', 'Springfield', 'Sturbridge', 'Taunton', 'Torrington', 'Woonsocket', 'Worcester']

    for i in range(len(city_list)):
        haversine(city_list[i], 'Greenfield', file)