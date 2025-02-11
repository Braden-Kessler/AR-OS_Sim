# https://www.latlong.net/

def longLat(name):
    mainList = []
    subList = []
    while True:
        while True:
            latLong = input()
            if latLong == "q":
                print(mainList)
                return
            elif latLong == "":
                test = input("confirm?")
                if test == "y":
                    break
                else:
                    continue
            else:
                latLong = latLong.split(',')
                try:
                    lat = float(latLong[0])
                    long = float(latLong[1])
                    subList.append([long, lat])
                except ValueError:
                    pass
        print(subList)
        mainList.append(subList)
        subList = []


if __name__ == '__main__':
    longLat('PyCharm')

