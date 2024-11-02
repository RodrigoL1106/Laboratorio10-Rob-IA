import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y):
        # goal position
        self.goal_x = goal_x
        self.goal_y = goal_y
        # obstacle positions
        self.obs_x = obs_x
        self.obs_y = obs_y
        # start position
        self.r_x = [start_x]
        self.r_y = [start_y]
        # latent space
        self.out_x = []
        self.out_y = []
        # map the latent space
        for o_x, o_y in zip(obs_x, obs_y):
            # possible 8 movements (include diagonals)
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                # validate the latent space
                cand_x, cand_y = o_x + add_x, o_y + add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    # if the latent space position is within the obstacle -> discard
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    # if the location is outside the obstacle, the latent space position is considered part of.
                    self.out_x.append(cand_x)
                    self.out_y.append(cand_y)

    def mov_normal(self):
        # difference between the goal and current position
        # 0, not move
        # positive, move forward
        # negative, move reverse
        return self.r_x[-1] + np.sign(self.goal_x - self.r_x[-1]), self.r_y[-1] + np.sign(self.goal_y - self.r_y[-1])

    def mov_to_next_obs(self, visited_x, visited_y):
    # no diagonal movements (1,0) (0,1) (-1,0) (0,-1)
        for add_x, add_y in zip([0, 1, 0, -1], [1, 0, -1, 0]):
            # update movement
            c_x, c_y = self.r_x[-1] + add_x, self.r_y[-1] + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                # validate movement, comparing the updated movement with the movement from out x_y list
                if c_x == _x and c_y == _y:
                    for v_x, v_y in zip(visited_x, visited_y):
                        # validate movement, comparing the updated movement with the visited movement
                        if c_x == v_x and c_y == v_y:
                            use_pt = False
                            break
                    # if update movement is not equal visited return the updated movement
                    if use_pt:
                        return c_x, c_y, False
                # use the next possible movement
                if not use_pt:
                    break
        # this position was visited before
        return self.r_x[-1], self.r_y[-1], True
                
    def bug0(self):
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

        mov_dir = 'normal'

        cand_x, cand_y = -np.inf, -np.inf

        # current position is a latent space
        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []

        while True:
            # achieve the goal position
            if self.r_x[-1] == self.goal_x and self.r_y[-1] == self.goal_y:
                break

            # move one step
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()

            # move in the latent space close to the obstacle
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)

            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    # if the next position is part of the latent space
                    if cand_x == x_ob and cand_y == y_ob:
                        # add the last position in the path (r_x, r_y)
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        # clean the visited position list because the vehicle detect a new obstacle
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        # the type of movement change
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)

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
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
                    visited_x.append(cand_x), visited_y.append(cand_y)
            if show_animation:
                print(f'{mov_dir},{cand_x},{cand_y}')
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)

        if show_animation:
            plt.show()

    def bug1(self):
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

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
            # achieve the goal position
            if self.r_x[-1] == self.goal_x and self.r_y[-1] == self.goal_y:
                break

            # move one step
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()

            # move in the latent space close to the obstacle
            if mov_dir == 'obs':
                cand_x, cand_y, back_to_start = self.mov_to_next_obs(visited_x, visited_y)

            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    # if the next position is part of the latent space
                    if cand_x == x_ob and cand_y == y_ob:
                        # add the last position in the path (r_x, r_y)
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        # clean the visited position list because the vehicle detect a new obstacle
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        dist = np.inf
                        back_to_start = False
                        second_round = False
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)

            elif mov_dir == 'obs':
                # distance between the target and actual position
                d = np.linalg.norm(np.array([cand_x, cand_y]) - np.array([self.goal_x, self.goal_y]))
                if d < dist and not second_round:
                    # find the lowest distance and save the position in exit_x, exit_y
                    exit_x, exit_y = cand_x, cand_y
                    dist = d
                if back_to_start and not second_round:
                    # reach again first position in front of the obstacle
                    second_round = True
                    # delete all the previous visited positions from the actual path
                    # during the obs move the visited position = actual path (r_x, r_y)
                    del self.r_x[-len(visited_x):]
                    del self.r_y[-len(visited_y):]
                    visited_x[:], visited_y[:] = [], []
                # save the position in the actual path and visited positions
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)

                # then to reach again first position
                # move on obs mode until reach the position with lowest distance to the target position
                # when the robot reaches exit_x exit_y, the robot can continue a normal movement mode
                if cand_x == exit_x and cand_y == exit_y and second_round:
                    mov_dir = 'normal'

            if show_animation:
                print(f'{mov_dir},{cand_x},{cand_y}')
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)

        if show_animation:
            plt.show()




    def bug2(self):
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf

        straight_x, straight_y = [self.r_x[-1]], [self.r_y[-1]]
        hit_x, hit_y = [], []

        while True:
            # the straight path reaches the goal
            if straight_x[-1] == self.goal_x and straight_y[-1] == self.goal_y:
                break

            # move to the target
            c_x = straight_x[-1] + np.sign(self.goal_x - straight_x[-1])
            c_y = straight_y[-1] + np.sign(self.goal_y - straight_y[-1])

            for x_ob, y_ob in zip(self.out_x, self.out_y):
                if c_x == x_ob and c_y == y_ob:
                    # collide with latent space, save the position in hit_x, hit_y
                    hit_x.append(c_x), hit_y.append(c_y)
                    break
            straight_x.append(c_x), straight_y.append(c_y)

        # current position is a latent space
        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []

        while True:
            # achieve the goal position
            if self.r_x[-1] == self.goal_x and self.r_y[-1] == self.goal_y:
                break

            # move one step
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()

            # move in the latent space close to the obstacle
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    # if the next position is part of the latent space
                    if cand_x == x_ob and cand_y == y_ob:
                        # add the last position in the path (r_x, r_y)
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        # clean the visited position list because the vehicle detects a new obstacle
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        # delete the first hit position
                        # the first contact of the robot to latent space is part of the hit positions
                        del hit_x[0]
                        del hit_y[0]
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)

            elif mov_dir == 'obs':
                # save the position in the actual path and visited positions
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                for i_x, i_y in zip(range(len(hit_x)), range(len(hit_y))):
                    # search the next hit position, which helps to get out of the latent space
                    if cand_x == hit_x[i_x] and cand_y == hit_y[i_y]:
                        del hit_x[i_x]
                        del hit_y[i_y]
                        mov_dir = 'normal'
                        break
            if show_animation:
                print(f'{mov_dir},{cand_x},{cand_y},{hit_x},{hit_y}')
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
            
        if show_animation:
            plt.show()
            


def main():
    # set obstacle positions
    o_x, o_y = [], []

    s_x = 0.0
    s_y = 0.0
    g_x = 230.0
    g_y = 70.0

    for i in range(80, 40):
        for j in range(80, 40):
            o_x.append(i)
            o_y.append(j)

    for i in range(60, 100):
        for j in range(40, 80):
            o_x.append(i)
            o_y.append(j)

    for i in range(120, 140):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)

    for i in range(80, 140):
        for j in range(0, 20):
            o_x.append(i)
            o_y.append(j)

    for i in range(0, 20):
        for j in range(60, 100):
            o_x.append(i)
            o_y.append(j)

    my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
    my_Bug.bug1()


if __name__ == '__main__':
    main()
