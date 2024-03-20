import numpy as np

# Open the text file for reading
with open('./result.txt', 'r') as file:
# with open('./result_all.txt', 'r') as file:
    # Read the contents of the file
    contents = file.read()

    # Split the contents by "------"
    data = contents.split("---------------")

    # Iterate over the data
    for item in data:
        means = []
        time = []
        # number is following "drone_num: "
        if "drone_num: " not in item:
            continue
        number = int(item.split("drone_num: ")[1].strip().split("\n")[0])
        ratio = float(item.split("noise_rate: ")[1].strip().split("\n")[0])
        solver = item.split("solver_type: ")[1].strip().split("\n")[0]
        if "%%%%%%%" not in item:
            continue
        drone = item.split("%%%%%%%")
        for d in drone:
            if "angle_result" not in d:
                continue
            # Extract the numbers after "angle_result"
            angles = d.split("angle_result: ")[1].strip().split("\n")[0]
            angle = angles.split(" ")
            # remove last item
            angle = np.array(angle[:-1], dtype=np.float64)
            # Extract the numbers after "angle_result_real"
            angles_real = d.split("angle_result_real: ")[1].strip().split("\n")[0]
            angle_real = angles_real.split(" ")
            angle_real = np.array(angle_real[:-1], dtype=np.float64)
            # find indice corresponding to 0
            idx = np.where(angle_real != 0)
            #take only non-zero elements
            angle = angle[idx]
            angle_real = angle_real[idx]
            # calculate mean absolute error, normalize to -pi to pi
            error = np.abs(angle - angle_real)
            error = np.where(error > np.pi, 2 * np.pi - error, error)
            mean_error = np.mean(error)
            means.append(mean_error)
            time.append(float(d.split("time: ")[1].strip().split("\n")[0]))

        # Calculate the mean of the means
        mean = np.mean(means)
        time = np.mean(time)
        # print(f"{number} drones with noise rate {ratio}, mean error: {mean}")
        print(f"{solver}: {number} drones with noise rate {ratio}, mean error: {mean}, time: {time}")
