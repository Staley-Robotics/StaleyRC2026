import util

# calculator = util.LaunchCalc()


while True: 
    distance = input("Input test distance in meters (Press 'Enter' to exit): ")
    if distance == "":
        break
    target_speed = util.LaunchCalc.get_target_speed(float(distance))
    print(f"Target speed for {distance} meters is {target_speed} rotations per second")