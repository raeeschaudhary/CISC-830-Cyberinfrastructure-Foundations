import matplotlib.pyplot as plt

# Sample data for three lines
X = [10, 100, 1000, 10000, 100000, 1000000]
BF = [0.001, 0.001, 0.004, 0.226, 20.510, 2164.709]
BF_OMP = [0.003, 0.003, 0.011, 0.054, 4.082, 398.410]
BF_GPU = [0.049, 0.049, 0.051, 0.058, 0.194, 7.419]
KDTREE = [0.003, 0.003, 0.003, 0.023, 0.222, 2.579]
KDTREE_OMP = [0.002, 0.004, 0.004, 0.028, 0.107, 1.240]

# Plotting the lines with circles at data points
plt.plot(X, BF, marker='o', label='Brute Force ')
plt.plot(X, BF_OMP, marker='o', label='Brute Force OMP')
plt.plot(X, BF_GPU, marker='o', label='Brute Force GPU')
plt.plot(X, KDTREE, marker='o', label='KDTree')
plt.plot(X, KDTREE_OMP, marker='o', label='KDTree OMP')

# Adding title and labels
plt.title('Execution Time VS Number of Points for Chamfer Distance')
plt.xlabel('Number of Points')
plt.ylabel('Execution Time (s)')

# Adding legend
plt.legend()

# Displaying the plot
plt.grid(True)
plt.show()
