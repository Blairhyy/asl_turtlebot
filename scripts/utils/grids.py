import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque


# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

        

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.1):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = probs
        self.window_size = window_size
        self.thresh = thresh
        probs_2d = np.array(self.probs).reshape(self.width, -1)

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def is_free(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        p_total = 1.0
        lower = -int(round((self.window_size-1)/2))
        upper = int(round((self.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x, y = self.snap_to_grid([state[0] + dx * self.resolution, state[1] + dy * self.resolution])
                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
        # print(p_total, self.thresh, "isfree stoch")
        return (1.0-p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(len(self.probs)):
            # convert i to (x,y)
            gy = int(i/self.width)
            gx = i % self.width
            x = gx * self.resolution + self.origin_x
            y = gy * self.resolution + self.origin_y
            if not self.is_free((x,y)):
                pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')

class DilatedDetOccupancyGrid2D(StochOccupancyGrid2D):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.1):
        super().__init__(resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh = thresh)
        #robot size
        self.dilation = 1
        self.dimX, self.dimY = int(self.width//self.resolution), int(self.height//self.resolution)
        self.grid = np.zeros((self.dimX, self.dimY), dtype = bool)
        todo = deque()
        for i in range(self.dimX):
            for j in range(self.dimY):
                if not super().is_free([i*self.resolution, j*self.resolution]):
                    todo.append((i, j))
                    self.grid[i][j] = 1
        step = 0
        while todo or step > self.dilaton/self.resolution:
            level = len(todo)
            for _ in range(level):
                x, y = todo.pop_left()
                for i,j in [[1,0],[0,1],[-1,0],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]]:
                    if 0 <= x+i < self.dimX and 0 <= y+j < self.dimY and self.self.grid[x+i][y+j] == 0:
                        self.grid[x+i][y+j] = 1
                        todo.append((x+i, y+j))
            step += 1
    def is_free(self, state):
        x = round(state[0]/self.resolution)
        y = round(state[1]/self.resolution)
        return not self.grid[x][y]
    
    def plot(self):
        plt.imshow(self.grid)


class FilterDilation(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.1):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = probs
        self.window_size = window_size
        self.thresh = thresh

        # dilation = int(0.08//self.resolution)
        dilation = 6
    
        probs_2d = np.array(self.probs).reshape(self.width, -1)
        probs_2d_pad = np.pad(probs_2d, ((dilation//2, dilation//2), (dilation//2, dilation//2)), 'constant', constant_values = (0, 0))
        self.dilated_prob_2d = np.zeros((len(probs_2d), len(probs_2d[0])))
        for i in range (len(probs_2d)):
            for j in range (len(probs_2d[0])):
                submatrix = probs_2d_pad[i:i+dilation, j:j+dilation]
                if (1-np.max(submatrix)) < self.thresh :
                    self.dilated_prob_2d[i][j] = 1
        self.probs_2d = probs_2d
                
    
    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))
        
    def is_free(self,state):
        x, y = self.snap_to_grid(state)
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
            p = self.dilated_prob_2d[grid_x][grid_y]
            return p == 0
        
        return False