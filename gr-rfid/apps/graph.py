import sys

from pandas import DataFrame
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

if(len(sys.argv) < 3):
    print("Execute the program with at least 2 factors.")
    print("1: the number of slots you want to analyze")
    print("2: the number of clusters you want to analyze")
else:
    slot_number = int(sys.argv[1])
    cluster_number = int(sys.argv[2])
    folder_path = "debug_data/RN16_iq/"

    for round_num in range(1, slot_number+1):
        file_path = folder_path + str(round_num) + "_1_"

        with open(file_path+"i", 'r') as f:
            data_i = f.read().split(' ')
        with open(file_path+"q", 'r') as f:
            data_q = f.read().split(' ')

        data_i.pop()
        data_q.pop()

        data_i = list(map(float, data_i))
        data_q = list(map(float, data_q))

        df = DataFrame(columns=['x','y'])

        for idx in range(0, len(data_i)):
            df.loc[idx] = [data_i[idx], data_q[idx]]
      
        kmeans = KMeans(n_clusters=cluster_number).fit(df)
        centroids = kmeans.cluster_centers_

        plt.scatter(df['x'], df['y'], c= kmeans.labels_.astype(float), s=50, alpha=0.5)
        plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=50)
        plt.savefig(folder_path + "graph/" + str(round_num) + ".png")
        plt.clf()

        print("Progressing.. (" + str(round_num) + "/" + str(slot_number) + ")")
