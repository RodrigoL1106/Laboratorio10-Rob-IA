import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y):
        #goal position
        self.goal_x = goal_x
        self.goal_y = goal_y
        #obstacle positions
        self.obs_x = obs_x
        self.obs_y = obs_y
        #start position
        self.r_x = [start_x]
        self.r_y = [start_y]
        #latent space
        self.out_x = []
        self.out_y = []

        for o_x, o_y in zip(obs_x, obs_y):
            #posible 8 movements (include diagonals)
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                self.out_x.append(o_x + add_x)
                self.out_y.append(o_y + add_y)
                #validate the latent space
                cand_x, cand_y = o_x + add_x, o_y + add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    #if the location is outside the obstacle, space position is considered part of.
                    self.out_x.append(cand_x), self.out_y.append(cand_y)

    def mov_normal(self):
        # diference between the goal and current possition
        # 0, not move
        # positive, move forward
        # negative, move reverse
        return self.r_x[-1] + np. sign(self.goal_x - self.r_x[-1]), self. r_y[-1] + np.sign(self.goal_y - self. r_y[-1])

    def mov_to_next_obs(self, visited_x, visited_y) :
        # no diagonal movements (1,0) (0,1) (-1,0) (0 .- 1)
        for add_x, add_y in zip([0, 1,0,-1],[1,0,-1,0]):
        # update movement
            c_x, c_y = self.r_x[-1] + add_x, self.r_y[-1] + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                # validate movement, comparing the updated movement with the movement from out x_y list
                if c_x == _x and c_y == _y:
                    for v_x, v_y in zip(visited_x, visited_y) :
                    # validate movement, comparing the updated movement withn the visited movement
                        if c_x == v_x and c_y == v_y:
                            use_pt = False
                            break
                    # if update movement is not equal visited return the updated movement
                    if use_pt:
                        return c_x, c_y, False
                # use the next possible movement
                if not use_pt:
                    break
        return self.r_x[-1], self.r_y[-1], True

    def bug0(self):
        """
        Greedy algorithm where you move towards goal until you hit an obstacle. Then you go around it
        (pick an arbitrary direction), until it is possible for you to start moving towards goal in a greedy manner again
        """
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf

        # current position is a latent space
        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            # achive the goal position
            if self.r_x[-1] == self.goal_x and self.r_y[-1] == self.goal_y:
                break

            # move one step
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()

            # move in the latent space close to the obstacle
            if mov_dir == 'obs':
                cand_x, cand_y, = self.mov_to_next_obs(visited_x, visited_y)
            
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    # if the next position is part of the latent space
                    if cand_x == x_ob and cand_y == y_ob:
                        # add the last position in the path (r_x, r_y)
                        self.r_x.append(cand_x)
                        self.r_y.append(cand_y)
                        # clean the visited position list because the vehicle detect a new obstacle
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x)
                        visited_y.append(cand_y)
                        # the type of movement change
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x)
                    self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                can_go_normal = True
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    # the next step is part of the obstacle
                    if self.mov_normal()[0] == x_ob and self.mov_normal()[1] == y_ob:
                        can_go_normal = False
                        break
                if can_go_normal:
                    mov_dir = 'normal'
                else:
                    self.r_x.append(cand_x)
                    self.r_y.append(cand_y)
                    visited_x.append(cand_x)
                    visited_y.append(cand_y)

    def bug1(self):
        """
        Move towards goal in a greedy manner. When you hit an obstacle, you go around it and
        back to where you hit the obstacle initially. Then, you go to the point on the obstacle that is
        closest to your goal and you start moving towards goal in a greedy manner from that new point.
        """
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        exit_x, exit_y = -np.inf, -np.inf
        dist = np.inf
        back_to_start = False
        second_round = False

        # current position is a latent space
        for xob, yob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == xob and self.r_y[-1] == yob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            # achive the goal position
            if self.r_x[-1] == self.goal_x and self.r_y[-1] == self.goal_y:
                break

            # move one step
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()

            # move in the latent space close to the obstacle
            if mov_dir == 'obs':
                cand_x, cand_y, back_to_start = self.mov_to_next_obs (visited_x, visited_y)
            
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    # if the next position is part of the latent space
                    if cand_x == x_ob and cand_y == y_ob:
                        # add the last position in the path (r_x, r_y)
                        self.r_x.append(cand_x)
                        self.r_y.append(cand_y)
                        # clean the visited position list because the vehicle detect a new obstacle
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x)
                        visited_y.append(cand_y)
                        mov_dir = 'obs'
                        dist = np.inf
                        back_to_start = False
                        second_round = False
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x)
                    self.r_y.append(cand_y)

    def bug2(self):
        """
        Move towards goal in a greedy manner. When you hit an obstacle, you go around it and
        keep track of your distance from the goal. If the distance from your goal was decreasing before
        and now it starts increasing, that means the current point is probably the closest point to the
        goal (this may or may not be true because the algorithm doesn't explore the entire boundary around the obstacle).
        So, you depart from this point and continue towards the goal in a greedy manner
        """
        
def main():

if __name__ == '__main__':
    main()