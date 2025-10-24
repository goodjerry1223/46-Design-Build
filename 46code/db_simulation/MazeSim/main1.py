import numpy as np
from simulator.map_loader import load_map
from simulator.lidar_sim import simulate_lidar_with_distances
from planner.a_star import a_star_search
from visualization.map_viewer import MapViewer
from hardware.hardware_manager import HardwareManager
import config
import time
import matplotlib.pyplot as plt
import numpy as np

STEP_SIZE = 1

def move_towards(cur_pos, target_pos, step_size=STEP_SIZE):
    dx = target_pos[0] - cur_pos[0]
    dy = target_pos[1] - cur_pos[1]
    dist = np.hypot(dx, dy)

    if dist < step_size * 0.8:
        return target_pos

    move_dist = min(dist, step_size)
    nx = cur_pos[0] + dx / dist * move_dist
    ny = cur_pos[1] + dy / dist * move_dist
    return (nx, ny)

# 全局变量存储当前状态
current_viewer = None
current_map_file = 'map1.json'
hardware_manager = None

def initialize_map(map_file):
    """初始化地图和相关状态"""
    global current_viewer, current_map_file, hardware_manager
    current_map_file = map_file
    
    # 如果已有viewer，关闭它
    if current_viewer is not None:
        plt.close(current_viewer.fig)
    
    # 如果已有硬件管理器，关闭它
    if hardware_manager is not None:
        hardware_manager.shutdown()
    
    maze, start, _ = load_map(map_file)
    explored = -np.ones_like(maze)
    explored[start] = 0
    
    slam_shape = (maze.shape[0]*10, maze.shape[1]*10)
    
    # 初始化硬件管理器
    hardware_manager = HardwareManager()
    if not hardware_manager.initialize(maze, start):
        print("硬件管理器初始化失败，程序可能无法正常运行")
    
    # 创建新的viewer，传入地图切换回调函数
    current_viewer = MapViewer(np.where(explored == -1, 0.5, explored), 
                              slam_map_shape=slam_shape, 
                              map_switch_callback=switch_map)
    
    return maze, start, explored, slam_shape, current_viewer

def switch_map(map_file):
    """地图切换回调函数"""
    print(f"正在切换到地图: {map_file}")
    run_exploration(map_file)

def run_exploration(map_file):
    """运行地图探索"""
    # 位置噪声标准差 (格点单位)
    noise_std_dev_pos = 0.8
    # 角度噪声标准差 (弧度)
    noise_std_dev_theta = 0.2

    maze, start, explored, slam_shape, viewer = initialize_map(map_file)

    cur_pos = (float(start[0]), float(start[1]))
    slam_traj = []
    slam_pose = None
    all_path = [start]
    factor = slam_shape[0] // maze.shape[0]
    optimal_path = None
    print("主动SLAM探索开始...")

    viewer.set_phase("Active Exploration")

    update_counter = 0
    while True:
        # 更新硬件管理器的环境信息（仅在仿真模式下有效）
        hardware_manager.update_simulation_environment(maze, (int(round(cur_pos[0])), int(round(cur_pos[1]))))
        
        # 获取雷达数据（支持硬件和仿真模式）
        lidar_visible, lidar_angles, lidar_distances = hardware_manager.get_lidar_data()
        
        # 如果没有获取到数据，使用备用方案
        if not lidar_angles or not lidar_distances:
            # 更新硬件管理器的环境信息
            hardware_manager.update_simulation_environment(maze, (int(round(cur_pos[0])), int(round(cur_pos[1]))))
            
            # 获取雷达数据
            lidar_visible, lidar_angles, lidar_distances = hardware_manager.get_lidar_data()
            
            # 备用方案
            if not lidar_angles or not lidar_distances:
                lidar_visible, lidar_angles, lidar_distances = simulate_lidar_with_distances((int(round(cur_pos[0])), int(round(cur_pos[1]))), maze, scan_size=32, max_dist=6)

        for pt in lidar_visible:
            explored[pt] = maze[pt]

        slam_map = np.full(slam_shape, 0.5)
        for i in range(maze.shape[0]):
            for j in range(maze.shape[1]):
                if explored[i, j] == 1:
                    slam_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor] = 0
                elif explored[i, j] == 0:
                    slam_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor] = 1

        # 更新SLAM轨迹，添加噪声
        slam_x_base = cur_pos[0]*factor + factor//2
        slam_y_base = cur_pos[1]*factor + factor//2
        
        slam_x_noisy = slam_x_base + np.random.normal(0, noise_std_dev_pos)
        slam_y_noisy = slam_y_base + np.random.normal(0, noise_std_dev_pos)
        
        slam_traj.append((slam_x_noisy, slam_y_noisy))
        
        if len(slam_traj) > 1:
            dx = slam_traj[-1][0] - slam_traj[-2][0]
            dy = slam_traj[-1][1] - slam_traj[-2][1]
            slam_theta_base = np.arctan2(dy, dx) if dx or dy else 0
            slam_theta_noisy = slam_theta_base + np.random.normal(0, noise_std_dev_theta)
        else:
            slam_theta_noisy = 0
        slam_pose = ((slam_x_noisy, slam_y_noisy), slam_theta_noisy)

        h, w = maze.shape
        candidates = []
        for i in range(h):
            for j in range(w):
                if explored[i, j] == 0:
                    for dx_cand, dy_cand in [(-1,0),(1,0),(0,-1),(0,1)]:
                        ni, nj = i+dx_cand, j+dy_cand
                        if 0<=ni<h and 0<=nj<w and explored[ni, nj]==-1:
                            candidates.append((i, j))
                            break
        if not candidates:
            print("探索完毕。")
            exits = [(0, j) for j in range(w) if maze[0, j] == 0 and (0, j) != start] + \
                    [(h-1, j) for j in range(w) if maze[h-1, j] == 0 and (h-1, j) != start] + \
                    [(i, 0) for i in range(h) if maze[i, 0] == 0 and (i, 0) != start] + \
                    [(i, w-1) for i in range(h) if maze[i, w-1] == 0 and (i, w-1) != start]

            minlen = float('inf')
            for exit_point in exits:
                path = a_star_search(np.where(explored==-1, 1, explored), start, exit_point)
                if path and len(path) < minlen:
                    minlen = len(path)
                    optimal_path = path
            break

        grid_cur = (int(round(cur_pos[0])), int(round(cur_pos[1])))
        goal = min(candidates, key=lambda pt: abs(pt[0]-grid_cur[0])+abs(pt[1]-grid_cur[1]))
        astar_path = a_star_search(np.where(explored==-1, 1, explored), grid_cur, goal)
        if astar_path is None or len(astar_path) < 2:
            print("路径规划失败，窗口不自动关闭。")
            for _ in range(50):
                viewer.update(
                    (int(round(cur_pos[0])), int(round(cur_pos[1]))),
                    path=all_path,
                    explore_map=np.where(explored==-1, 0.5, explored),
                    visible_cells=lidar_visible,
                    slam_traj=slam_traj,
                    slam_map=slam_map,
                    slam_pose=slam_pose,
                    lidar_angles=lidar_angles,
                    lidar_distances=lidar_distances,
                    optimal_path=optimal_path
                )
                time.sleep(0.1)
            plt.ioff()
            plt.show()
            return

        path_idx = 1
        while path_idx < len(astar_path):
            next_grid = astar_path[path_idx]
            new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
            cur_pos = new_pos

            lidar_visible, lidar_angles, lidar_distances = simulate_lidar_with_distances((int(round(cur_pos[0])), int(round(cur_pos[1]))), maze, scan_size=32, max_dist=6)

            for pt in lidar_visible:
                explored[pt] = maze[pt]

            slam_map = np.full(slam_shape, 0.5)
            for i in range(maze.shape[0]):
                for j in range(maze.shape[1]):
                    if explored[i, j] == 1:
                        slam_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor] = 0
                    elif explored[i, j] == 0:
                        slam_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor] = 1

            # 更新SLAM轨迹，添加噪声
            slam_x_base = cur_pos[0]*factor + factor//2
            slam_y_base = cur_pos[1]*factor + factor//2
            
            slam_x_noisy = slam_x_base + np.random.normal(0, noise_std_dev_pos)
            slam_y_noisy = slam_y_base + np.random.normal(0, noise_std_dev_pos)
            
            slam_traj.append((slam_x_noisy, slam_y_noisy))
            
            if len(slam_traj) > 1:
                dx = slam_traj[-1][0] - slam_traj[-2][0]
                dy = slam_traj[-1][1] - slam_traj[-2][1]
                slam_theta_base = np.arctan2(dy, dx) if dx or dy else 0
                slam_theta_noisy = slam_theta_base + np.random.normal(0, noise_std_dev_theta)
            else:
                slam_theta_noisy = 0
            slam_pose = ((slam_x_noisy, slam_y_noisy), slam_theta_noisy)

            if np.linalg.norm(np.array(cur_pos) - np.array(next_grid)) < STEP_SIZE * 0.6:
                path_idx += 1

            if np.linalg.norm(np.array(cur_pos) - np.array(goal)) < STEP_SIZE * 0.8:
                break

            if (int(round(cur_pos[0])), int(round(cur_pos[1]))) != all_path[-1]:
                all_path.append((int(round(cur_pos[0])), int(round(cur_pos[1]))))

            viewer.update(
                (int(round(cur_pos[0])), int(round(cur_pos[1]))),
                path=all_path,
                explore_map=np.where(explored==-1, 0.5, explored),
                visible_cells=lidar_visible,
                slam_traj=slam_traj,
                slam_map=slam_map,
                slam_pose=slam_pose,
                lidar_angles=lidar_angles,
                lidar_distances=lidar_distances,
                optimal_path=optimal_path
            )
            update_counter += 1
            time.sleep(0.005)

        if (int(round(cur_pos[0])), int(round(cur_pos[1]))) != all_path[-1]:
            all_path.append((int(round(cur_pos[0])), int(round(cur_pos[1]))))

    # 2. 回到起点
    print("探索完毕，回到起点")
    viewer.set_phase("Returning to Start (After Exploration)")
    lidar_visible = []
    lidar_angles = []
    lidar_distances = []
    cur_goal = start
    cur_grid = (int(round(cur_pos[0])), int(round(cur_pos[1])))
    return_path = a_star_search(np.where(explored==-1, 1, explored), cur_grid, cur_goal)
    if not return_path or len(return_path) < 2:
        print("回起点路径规划失败！窗口不自动关闭。")
        for _ in range(50):
            viewer.update(
                (int(round(cur_pos[0])), int(round(cur_pos[1]))),
                path=all_path,
                explore_map=np.where(explored==-1, 0.5, explored),
                visible_cells=[],
                slam_traj=slam_traj,
                slam_map=slam_map,
                slam_pose=slam_pose,
                lidar_angles=[],
                lidar_distances=[],
                optimal_path=optimal_path
            )
            time.sleep(0.1)
        plt.ioff()
        plt.show()
        return
    return_path_idx = 1
    while return_path and return_path_idx < len(return_path):
        next_grid = return_path[return_path_idx]
        # 计算目标方向和距离
        target_pos = next_grid
        dx = target_pos[0] - cur_pos[0]
        dy = target_pos[1] - cur_pos[1]
        target_angle = np.arctan2(dy, dx)
        
        # 计算控制指令（简化的控制逻辑）
        linear_vel = min(1.0, np.hypot(dx, dy))  # 线速度
        angular_vel = 0.0  # 角速度（简化处理）
        
        # 发送控制指令到硬件或模拟器
        hardware_manager.send_motion_command(linear_vel, angular_vel)
        
        # 模拟机器人移动（仅在仿真模式下更新位置）
        if not config.HARDWARE_MODE:
            new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
            cur_pos = new_pos
        else:
            # 硬件模式下，位置由SLAM系统提供
            # 这里可以添加从硬件获取真实位置的逻辑
            new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
            cur_pos = new_pos
        if np.linalg.norm(np.array(cur_pos) - np.array(next_grid)) < 0.2:
            return_path_idx += 1

        # 更新SLAM轨迹，添加噪声
        slam_x_base = cur_pos[0]*factor + factor//2
        slam_y_base = cur_pos[1]*factor + factor//2
        
        slam_x_noisy = slam_x_base + np.random.normal(0, noise_std_dev_pos)
        slam_y_noisy = slam_y_base + np.random.normal(0, noise_std_dev_pos)
        
        slam_traj.append((slam_x_noisy, slam_y_noisy))
        
        if len(slam_traj) > 1:
            dx = slam_traj[-1][0] - slam_traj[-2][0]
            dy = slam_traj[-1][1] - slam_traj[-2][1]
            slam_theta_base = np.arctan2(dy, dx) if dx or dy else 0
            slam_theta_noisy = slam_theta_base + np.random.normal(0, noise_std_dev_theta)
        else:
            slam_theta_noisy = 0
        slam_pose = ((slam_x_noisy, slam_y_noisy), slam_theta_noisy)

        if (int(round(cur_pos[0])), int(round(cur_pos[1]))) != all_path[-1]:
            all_path.append((int(round(cur_pos[0])), int(round(cur_pos[1]))))

        # 实时更新显示
        viewer.update(
            (int(round(cur_pos[0])), int(round(cur_pos[1]))),
            path=all_path,
            explore_map=np.where(explored==-1, 0.5, explored),
            visible_cells=[],
            slam_traj=slam_traj,
            slam_map=slam_map,
            slam_pose=slam_pose,
            lidar_angles=[],
            lidar_distances=[],
            optimal_path=optimal_path
        )
        if np.linalg.norm(np.array(cur_pos) - np.array(cur_goal)) < 0.2:
            break
        time.sleep(0.01)

    # 3. 从起点前往终点
    h, w = maze.shape
    exits = []
    for j in range(w):
        if maze[0, j] == 0 and (0, j) != start:
            exits.append((0, j))
        if maze[h-1, j] == 0 and (h-1, j) != start:
            exits.append((h-1, j))
    for i in range(h):
        if maze[i, 0] == 0 and (i, 0) != start:
            exits.append((i, 0))
        if maze[i, w-1] == 0 and (i, w-1) != start:
            exits.append((i, w-1))

    minlen = float('inf')
    final_goal = None
    for exit_point in exits:
        path = a_star_search(np.where(explored==-1, 1, explored), start, exit_point)
        if path and len(path) < minlen:
            minlen = len(path)
            final_goal = exit_point

    print("从起点走向终点:", (final_goal[1],final_goal[0]))
    viewer.set_phase("Going to Target")
    cur_goal = final_goal
    cur_grid = (int(round(cur_pos[0])), int(round(cur_pos[1])))
    goal_path = a_star_search(np.where(explored==-1, 1, explored), cur_grid, cur_goal)
    if not goal_path or len(goal_path) < 2:
        print("去终点路径规划失败！窗口不自动关闭。")
        for _ in range(50):
            viewer.update(
                (int(round(cur_pos[0])), int(round(cur_pos[1]))),
                path=all_path,
                explore_map=np.where(explored==-1, 0.5, explored),
                visible_cells=[],
                slam_traj=slam_traj,
                slam_map=slam_map,
                slam_pose=slam_pose,
                lidar_angles=[],
                lidar_distances=[],
                optimal_path=optimal_path
            )
            time.sleep(0.1)
        plt.ioff()
        plt.show()
        return
    goal_path_idx = 1
    while goal_path and goal_path_idx < len(goal_path):
        next_grid = goal_path[goal_path_idx]
        
        # 计算目标方向和距离
        target_pos = next_grid
        dx = target_pos[0] - cur_pos[0]
        dy = target_pos[1] - cur_pos[1]
        target_angle = np.arctan2(dy, dx)
        
        # 计算控制指令（简化的控制逻辑）
        linear_vel = min(1.0, np.hypot(dx, dy))  # 线速度
        angular_vel = 0.0  # 角速度（简化处理）
        
        # 发送控制指令到硬件或模拟器
        hardware_manager.send_motion_command(linear_vel, angular_vel)
        
        # 模拟机器人移动（仅在仿真模式下更新位置）
        if not config.HARDWARE_MODE:
            new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
            cur_pos = new_pos
        else:
            # 硬件模式下，位置由SLAM系统提供
            # 这里可以添加从硬件获取真实位置的逻辑
            new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
            cur_pos = new_pos
        if np.linalg.norm(np.array(cur_pos) - np.array(next_grid)) < 0.2:
            goal_path_idx += 1

        # 更新SLAM轨迹，添加噪声
        slam_x_base = cur_pos[0]*factor + factor//2
        slam_y_base = cur_pos[1]*factor + factor//2
        
        slam_x_noisy = slam_x_base + np.random.normal(0, noise_std_dev_pos)
        slam_y_noisy = slam_y_base + np.random.normal(0, noise_std_dev_pos)
        
        slam_traj.append((slam_x_noisy, slam_y_noisy))
        
        if len(slam_traj) > 1:
            dx = slam_traj[-1][0] - slam_traj[-2][0]
            dy = slam_traj[-1][1] - slam_traj[-2][1]
            slam_theta_base = np.arctan2(dy, dx) if dx or dy else 0
            slam_theta_noisy = slam_theta_base + np.random.normal(0, noise_std_dev_theta)
        else:
            slam_theta_noisy = 0
        slam_pose = ((slam_x_noisy, slam_y_noisy), slam_theta_noisy)

        if (int(round(cur_pos[0])), int(round(cur_pos[1]))) != all_path[-1]:
            all_path.append((int(round(cur_pos[0])), int(round(cur_pos[1]))))

        # 实时更新显示
        viewer.update(
            (int(round(cur_pos[0])), int(round(cur_pos[1]))),
            path=all_path,
            explore_map=np.where(explored==-1, 0.5, explored),
            visible_cells=[],
            slam_traj=slam_traj,
            slam_map=slam_map,
            slam_pose=slam_pose,
            lidar_angles=[],
            lidar_distances=[],
            optimal_path=optimal_path
        )
        if np.linalg.norm(np.array(cur_pos) - np.array(cur_goal)) < 0.2:
            break
        time.sleep(0.005)

    # 4. 终点停留2秒
    viewer.set_phase("Staying at Target")
    for _ in range(20):
        viewer.update(
            (int(round(cur_pos[0])), int(round(cur_pos[1]))),
            path=all_path,
            explore_map=np.where(explored==-1, 0.5, explored),
            visible_cells=[],
            slam_traj=slam_traj,
            slam_map=slam_map,
            slam_pose=slam_pose,
            lidar_angles=[],
            lidar_distances=[],
            optimal_path=optimal_path
        )
        time.sleep(0.1)

    # 5. 终点返回起点
    print("终点返回起点")
    viewer.set_phase("Returning to Start")
    cur_goal = start
    cur_grid = (int(round(cur_pos[0])), int(round(cur_pos[1])))
    return_path = a_star_search(np.where(explored==-1, 1, explored), cur_grid, cur_goal)
    if not return_path or len(return_path) < 2:
        print("终点返回起点路径规划失败！窗口不自动关闭。")
        for _ in range(50):
            viewer.update(
                (int(round(cur_pos[0])), int(round(cur_pos[1]))),
                path=all_path,
                explore_map=np.where(explored==-1, 0.5, explored),
                visible_cells=lidar_visible, 
                slam_traj=slam_traj,
                slam_map=slam_map,
                slam_pose=slam_pose,
                lidar_angles=lidar_angles,
                lidar_distances=lidar_distances,
                optimal_path=optimal_path
            )
            time.sleep(0.1)
        plt.ioff()
        plt.show()
        return
    return_path_idx = 1
    while return_path and return_path_idx < len(return_path):
        next_grid = return_path[return_path_idx]
        new_pos = move_towards(cur_pos, next_grid, step_size=STEP_SIZE)
        cur_pos = new_pos
        if np.linalg.norm(np.array(cur_pos) - np.array(next_grid)) < 0.2:
            return_path_idx += 1

        # 更新SLAM轨迹，添加噪声
        slam_x_base = cur_pos[0]*factor + factor//2
        slam_y_base = cur_pos[1]*factor + factor//2
        
        slam_x_noisy = slam_x_base + np.random.normal(0, noise_std_dev_pos)
        slam_y_noisy = slam_y_base + np.random.normal(0, noise_std_dev_pos)
        
        slam_traj.append((slam_x_noisy, slam_y_noisy))
        
        if len(slam_traj) > 1:
            dx = slam_traj[-1][0] - slam_traj[-2][0]
            dy = slam_traj[-1][1] - slam_traj[-2][1]
            slam_theta_base = np.arctan2(dy, dx) if dx or dy else 0
            slam_theta_noisy = slam_theta_base + np.random.normal(0, noise_std_dev_theta)
        else:
            slam_theta_noisy = 0
        slam_pose = ((slam_x_noisy, slam_y_noisy), slam_theta_noisy)

        if (int(round(cur_pos[0])), int(round(cur_pos[1]))) != all_path[-1]:
            all_path.append((int(round(cur_pos[0])), int(round(cur_pos[1]))))

        # 实时更新显示
        viewer.update(
            (int(round(cur_pos[0])), int(round(cur_pos[1]))),
            path=all_path,
            explore_map=np.where(explored==-1, 0.5, explored),
            visible_cells=[],
            slam_traj=slam_traj,
            slam_map=slam_map,
            slam_pose=slam_pose,
            lidar_angles=[],
            lidar_distances=[],
            optimal_path=optimal_path
        )
        if np.linalg.norm(np.array(cur_pos) - np.array(cur_goal)) < 0.2:
            break
        time.sleep(0.005)

    # 停在起点2秒
    for _ in range(20):
        viewer.update(
            (int(round(cur_pos[0])), int(round(cur_pos[1]))),
            path=all_path,
            explore_map=np.where(explored==-1, 0.5, explored),
            visible_cells=[],
            slam_traj=slam_traj,
            slam_map=slam_map,
            slam_pose=slam_pose,
            lidar_angles=[],
            lidar_distances=[],
            optimal_path=optimal_path
        )
        time.sleep(0.1)

    print("全部任务完成，窗口等待手动关闭。")
    viewer.set_phase("Mission Complete")

    plt.ioff()
    plt.show()

def main():
    """主函数，启动程序"""
    global hardware_manager
    
    print("MazeSim 地图探索程序启动")
    print(f"当前模式: {'硬件模式' if config.HARDWARE_MODE else '仿真模式'}")
    print("点击右上角的Map1、Map2、Map3按钮可以切换地图")
    
    try:
        run_exploration('map1.json')
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        # 清理硬件资源
        if hardware_manager:
            print("正在关闭硬件管理器...")
            hardware_manager.shutdown()
        print("程序已退出")

if __name__ == "__main__":
    main()