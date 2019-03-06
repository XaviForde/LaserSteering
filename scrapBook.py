import numpy as np


img = np.array([[1, 3 ,4], [3,4,5], [7, 8, 8]])

print(img)

aphids = np.array([])

aphid = np.array([6, 7, 9])
aphids = np.append(aphids,aphid)

print(aphids)
aphid = np.array([6, 7, 9])
aphids = np.append(aphids,aphid)

aphids = np.reshape(aphids, (-1,3))



print(aphids)