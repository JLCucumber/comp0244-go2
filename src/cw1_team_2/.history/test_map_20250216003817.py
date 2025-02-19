from cw1_team_2.environment.map import Environment
from cw1_team_2.utils.visualization import plot_path
from cw1_team_2.utils.config import CONFIG

if __name__ == "__main__":
    env = Environment(CONFIG["map_file"])
    start = (0, 0)
    goal = (4, 4)
    path = [start, goal]  # Dummy path for visualization purposes
    plot_path(env.obstacles, start, goal, path, title="Environment Visualization")