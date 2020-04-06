#Problem 6: Filter GPS Data

readfile_path = "HW1/NMEA0183_Data.txt"
writefile_path = "HW1/gps_data.txt" 

data_file = open(readfile_path, "r")

data_list = []
gps_list = []

def write_gps_to_file(readfile, write_path):
    #converting file text to a list
    for line in data_file:
        stripped_line = line.strip()
        line_list = stripped_line.split()
        data_list.append(line_list)

    #filtering out gps data starting with IIGGA
    i = 0
    while True:
        if(data_list[i][0][:6] == '$IIGGA'):
            gps_list.append(data_list[i])
        i += 1
        if i > len(data_list) - 1:
            break

    gps_file = open(write_path, "w")

    #writing the filtered gps data to seperate gps file 
    j = 0
    while True:
        gps_file.writelines(gps_list[j])
        gps_file.write('\n')
        j += 1
        if j > len(gps_list) - 1:
            break   

if __name__ == "__main__":
    write_gps_to_file(data_file, writefile_path)