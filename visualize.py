import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib import cm

def plot_robot_data(x, y, robot_number, color_map):
    colors = [color_map(i) for i in range(len(x))]
    plt.scatter(x, y, c=colors, label=f'Robot {robot_number}')

def main():
    filename = 'plan.txt'  # Replace with the actual file path

    with open(filename, 'r') as file:
        lines = file.readlines()

    x_values = []
    y_values = []
    current_robot = None
    robot_color_maps = {}

    for line in lines:
        if line.startswith('Robot'):
            if current_robot is not None:
                color_map = robot_color_maps[current_robot]
                plot_robot_data(x_values, y_values, current_robot, color_map)

            current_robot = int(line.split()[1])
            x_values = []
            y_values = []

            # Use a different colormap for each robot
            robot_color_maps[current_robot] = cm.get_cmap('viridis')

        else:
            values = [float(val) for val in line.split()[:2]]
            x_values.append(values[0])
            y_values.append(values[1])

    if current_robot is not None:
        color_map = robot_color_maps[current_robot]
        plot_robot_data(x_values, y_values, current_robot, color_map)

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Robot Trajectories')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
