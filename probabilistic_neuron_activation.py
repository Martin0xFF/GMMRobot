import numpy as np

class HaramBayesian():

    def __init__(self, wmap = None, ctnum= None):
        if wmap is None:
            self.map = {
                0: "yellow",
                1: "green",
                2: "blue",
                3: "orange",
                4: "orange",
                5: "green",
                6: "blue",
                7: "orange",
                8: "yellow",
                9: "green",
                10: "blue"
            }
        else:
            self.map = wmap

        if ctnum is None:

            self.colour_to_num = {
                "blue": 0,
                "green": 1,
                "yellow": 2,
                "orange": 3,
                "nothing": 4
            }
        else:
            self.colour_to_num = ctnum
        self.n = len(self.map)
        self.x_k = np.ones((self.n,1)) / self.n #assume equal probability that we're everywhere
        '''
        0-2
        1-3
        ...
        10-12
        '''
        self.state_model = np.array([[0.85, 0.05, 0.05],
                                    [0.1, 0.9, 0.1],
                                    [0.05, 0.05, 0.85]])

        # implicitly fill in "nothing column"
        self.measurement_model = np.array([[0.9, 0.025, 0.025, 0.025, 0.025], #g
                                    [0.025, 0.9, 0.025, 0.025, 0.025], #o
                                    [0.025, 0.025, 0.9, 0.025, 0.025], #p
                                    [0.025, 0.025, 0.025, 0.9, 0.025], #y
                                    [0.2, 0.2, 0.2, 0.2, 0.2]]) #l

        # given u_k+1 and z_k+1. from that, estimate state x_k+1|x_k,u_k using state model
    def state_estimate(self, u_k1):
        temp = np.zeros((self.n,1))
        for i in range(len(self.x_k)):
            for k in range(-1,2):
                if i+k > len(self.x_k) - 1:
                    ind = (i+k) % len(self.x_k)
                else:
                    ind = i+k
                temp[i]  +=  self.state_model[(-k)+1][1 + u_k1]*self.x_k[ind]
        self.x_k = temp


    # also estimate x_k+1|z_0:k+1 with measurement model
    def measurement_update(self, z_k1):
        for i, state_estimate in enumerate(self.x_k):
            col = self.colour_to_num[self.map[i]]
            row = self.colour_to_num[z_k1]
            self.x_k[i] = state_estimate* self.measurement_model[row][col]
        self.x_k = self.x_k/np.sum(self.x_k)

    def step(self, u_k1, z_k1):
        self.state_estimate(u_k1)
        self.measurement_update(z_k1)
        return self.x_k
'''

hb = HaramBayesian()


uk = [
1,
1,
1,
1,
1,
1,
1,
1,
0,
1,
1,
1
]

zk = [
"orange",
"yellow",
"green",
"blue",
"nothing",
"green",
"blue",
"green",
"orange",
"yellow",
"green",
"blue"
]

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={'float_kind':float_formatter})

for i in range(len(zk)):
    step = hb.step(uk[i], zk[i])
    temp = ""
    for s in step:
        print s,
    print(temp)


'''