import sys

from pandas import DataFrame
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

if(len(sys.argv) < 4):
    print("Execute the program with at least 2 factors.")
    print("1: the name of folder you want to analyze")
    print("2: the number of slots you want to analyze")
    print("3: the number of clusters you want to analyze")
else:
    folder_name = sys.argv[1]
    slot_number = int(sys.argv[2])
    cluster_number = int(sys.argv[3])
    folder_path = "debug_data/" + folder_name + "/"

    for round_num in range(1, slot_number+1):
        file_path = folder_path + str(round_num) + "_1_"

        with open(file_path+"I", 'r') as f:
            data_i = f.read().split(' ')
        f.close()
        with open(file_path+"Q", 'r') as f:
            data_q = f.read().split(' ')
        f.close()

        data_i.pop()
        data_q.pop()

        data_i = list(map(float, data_i))
        data_q = list(map(float, data_q))

        df = DataFrame(columns=['x','y'])

        for idx in range(0, len(data_i)):
            df.loc[idx] = [data_i[idx], data_q[idx]]

        kmeans = KMeans(n_clusters=cluster_number).fit(df)
        centroids = kmeans.cluster_centers_
        f = open("graph/" + folder_name + "/centroids_" + str(round_num), 'w')
        for i in range(0, cluster_number):
            f.write(str(centroids[i][0]))
            f.write(" ")
            f.write(str(centroids[i][1]))
            f.write("\n")
        f.close()

        plt.scatter(df['x'], df['y'], c= kmeans.labels_.astype(float), s=50, alpha=0.5)
        plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=50)
        plt.savefig("graph/" + folder_name + "/" + str(round_num) + ".png")
        plt.clf()

        print("Progressing.. (" + str(round_num) + "/" + str(slot_number) + ")")
