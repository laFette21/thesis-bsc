from matplotlib import pyplot as plt

with open("../bin/out.txt", "r") as iF:
    for line in iF:
        line = line.strip().split()

        plt.plot(float(line[1]), float(line[2]), '.', color='blue')

plt.axis("equal")
plt.show()
