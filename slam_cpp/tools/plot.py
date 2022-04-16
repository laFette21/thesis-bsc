import sys

from matplotlib import pyplot as plt

if False:
    with open('../bin/kecso_poses_gt.txt', 'r') as iF:
        for line in iF:
            line = line.strip().split()
            plt.plot(float(line[0]), float(line[2]), '.', color='purple')

    with open('../bin/out.txt', 'r') as iF:
        for line in iF:
            line = line.strip().split()

            if line[0] == '###':
                break

            plt.plot(float(line[1]), float(line[3]), '.', color='black')
else:
    # with open('../bin/kecso_poses_gt.txt', 'r') as iF:
    #     for line in iF:
    #         line = line.strip().split()
    #         plt.plot(float(line[0]), float(line[1]), '.', color='purple')

    with open(sys.argv[1], 'r') as iF:
        line = ['===']

        while True:
            if line[0] == '===':
                for line in iF:
                    line = line.strip().split()

                    if line[0] == 'END':
                        exit(0)

                    if line[0] == '===':
                        continue

                    if line[0] == '###':
                        break

                    plt.plot(float(line[1]), float(line[2]), '.', color='orange')

                for line in iF:
                    line = line.strip().split()

                    if line[0] == '###':
                        break

                    plt.plot(float(line[1]), float(line[2]), 'o', color='green')

                for line in iF:
                    line = line.strip().split()

                    if line[0] == '###':
                        break

                    plt.plot(float(line[1]), float(line[2]), '.', color='black')

                for line in iF:
                    line = line.strip().split()

                    if line[0] == '===':
                        break
            
                    plt.plot(float(line[4]), float(line[5]), 'o', color='blue')
                    plt.plot(float(line[1]), float(line[2]), 'o', color='red')

                plt.axis("equal")
                plt.show()
