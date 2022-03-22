from matplotlib import pyplot as plt

with open("../bin/out.txt", "r") as iF:
    for line in iF:
        line = line.strip().split()

        if line[0] == "###":
            break

        plt.plot(float(line[1]), float(line[2]), '.', color='black')

    for line in iF:
        line = line.strip().split()

        plt.plot(float(line[4]), float(line[5]), 'o', color='blue')
        plt.plot(float(line[1]), float(line[2]), 'o', color='red')

plt.axis("equal")
plt.show()