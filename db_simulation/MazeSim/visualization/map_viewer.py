import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.widgets import Button

class MapViewer:
    def __init__(self, explore_map, slam_map_shape=(100, 150), map_switch_callback=None):
        # 现代化配色方案
        self.colors = {
            'background': '#2c3e50',
            'panel': '#34495e', 
            'robot': '#e74c3c',
            'explore_path': '#2ecc71',
            'optimal_path': '#3498db',
            'lidar': '#f39c12',
            'slam_path': '#9b59b6',
            'text': '#ecf0f1',
            'grid': '#7f8c8d',
            'radar_line': '#e67e22',
            'radar_fill': '#3498db'
        }
        
        # 存储地图切换回调函数
        self.map_switch_callback = map_switch_callback
        
        # 创建优化的布局：更大的窗口，合理的区域分配
        self.fig = plt.figure(figsize=(20, 12), facecolor=self.colors['background'])
        
        # 顶部数据栏 (居中，确保可见)
        self.info_ax = plt.subplot2grid((15, 18), (0, 4), colspan=10, rowspan=1)
        
        # 按钮区域 (右上角，独立一行避免与标题栏冲突)
        self.button_ax1 = plt.subplot2grid((15, 18), (1, 15), colspan=1, rowspan=1)
        self.button_ax2 = plt.subplot2grid((15, 18), (1, 16), colspan=1, rowspan=1)
        self.button_ax3 = plt.subplot2grid((15, 18), (1, 17), colspan=1, rowspan=1)
        
        # 主要显示区域 - 左边地图探索区域
        self.ax = plt.subplot2grid((15, 18), (2, 1), rowspan=13, colspan=5)
        
        # SLAM区域 - 中间（增加间距）
        self.slam_ax = plt.subplot2grid((15, 18), (2, 7), rowspan=13, colspan=5)
        
        # 右边区域 - 数据栏和雷达图（向下移动）
        self.data_ax = plt.subplot2grid((15, 18), (3, 12), rowspan=5, colspan=6)
        self.radar_ax = plt.subplot2grid((15, 18), (9, 12), rowspan=6, colspan=6)
        
        self.ax.set_aspect('equal')
        self.slam_ax.set_aspect('equal')
        self.radar_ax.set_aspect('equal')
        
        # 调整子图间距，避免重叠
        plt.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.05, 
                           wspace=0.20, hspace=0.25)
        
        # 设置样式
        for ax in [self.ax, self.slam_ax, self.data_ax, self.radar_ax]:
            ax.set_facecolor(self.colors['background'])
            ax.tick_params(colors=self.colors['text'])
            # 设置黑色边框
            ax.spines['bottom'].set_color('black')
            ax.spines['top'].set_color('black')
            ax.spines['right'].set_color('black')
            ax.spines['left'].set_color('black')
            ax.spines['bottom'].set_linewidth(2)
            ax.spines['top'].set_linewidth(2)
            ax.spines['right'].set_linewidth(2)
            ax.spines['left'].set_linewidth(2)
        
        # 设置标题
        self.data_ax.set_title('Real-time Data', color=self.colors['text'], fontsize=12, fontweight='bold')
        self.radar_ax.set_title('Lidar Radar View', color=self.colors['text'], fontsize=12, fontweight='bold')
        
        # 设置数据栏
        self._setup_data_panel()
        
        # 设置雷达图
        self._setup_radar_panel()
        
        # 信息面板相关属性
        self.start_time = time.time()
        self.current_phase = "Exploration Phase"
        self.total_distance = 0.0
        self.last_robot_pos = None
        
        # 初始化信息面板
        self._setup_info_panel()

        # 左侧探索区域 - 现代化样式
        self.wall_img = self.ax.imshow(explore_map, cmap='gray_r', vmin=0, vmax=1, interpolation='none', origin='lower')
        self.path_line, = self.ax.plot([], [], color=self.colors['explore_path'], linewidth=3, label='Exploration Path', alpha=0.8)
        self.optimal_line, = self.ax.plot([], [], color=self.colors['optimal_path'], linewidth=3, label='Optimal Path', alpha=0.9)
        self.robot_dot, = self.ax.plot([], [], 'o', color=self.colors['robot'], markersize=12, label='Robot', markeredgecolor='white', markeredgewidth=2)
        self.lidar_scatter = self.ax.scatter([], [], c=self.colors['lidar'], marker='s', s=80, alpha=0.6, label='Lidar Scan', edgecolors='white', linewidths=0.5)
        
        # -------- legend 创建已移到 update 动态分配 --------

        self.ax.set_title("Map Exploration & Path Planning", fontsize=14, color=self.colors['text'], fontweight='bold', pad=20)
        self.ax.tick_params(colors=self.colors['text'])
        self.ax.grid(True, color=self.colors['grid'], alpha=0.3)

        # 右侧SLAM区域 - 现代化样式
        self.slam_img = self.slam_ax.imshow(np.full(slam_map_shape, 0.5), cmap='gray', vmin=0, vmax=1, origin='lower')
        self.slam_path_line, = self.slam_ax.plot([], [], color=self.colors['slam_path'], linewidth=3, label='SLAM Trajectory', alpha=0.8)
        
        # -------- slam_legend 创建已移到 update 动态分配 --------

        self.slam_ax.set_title("SLAM Mapping & Localization", fontsize=14, color=self.colors['text'], fontweight='bold', pad=20)
        self.slam_ax.tick_params(colors=self.colors['text'])
        self.slam_ax.grid(True, color=self.colors['grid'], alpha=0.3)
        self.slam_arrow = None  # 用于方向箭头

        # 图例 handle 初始化
        self.legend_handle = None
        self.slam_legend_handle = None
        
        # 设置按钮
        self._setup_map_buttons()
        
        plt.ion()
        plt.show()
    
    def _setup_info_panel(self):
        """设置信息面板"""
        self.info_ax.set_facecolor(self.colors['panel'])
        self.info_ax.axis('off')
        
        # 初始信息文本
        info_text = (f"Current Phase: {self.current_phase}  |  "
                    f"Runtime: 0.0s  |  "
                    f"Distance: 0.00  |  "
                    f"Progress: 0.0%")
        
        self.info_text = self.info_ax.text(0.5, 0.5, info_text, 
                                          horizontalalignment='center', 
                                          verticalalignment='center',
                                          fontsize=12, color=self.colors['text'], 
                                          fontweight='bold',
                                          bbox=dict(boxstyle="round,pad=0.5", 
                                                   facecolor=self.colors['background'], 
                                                   edgecolor=self.colors['text'],
                                                   alpha=0.8))
    
    def _setup_map_buttons(self):
        """设置地图切换按钮"""
        # 设置按钮区域样式，添加明显的黑色边框
        for ax in [self.button_ax1, self.button_ax2, self.button_ax3]:
            ax.set_facecolor(self.colors['panel'])
            ax.axis('off')
            # 添加明显的黑色边框
            for spine in ax.spines.values():
                spine.set_edgecolor('black')
                spine.set_linewidth(3)
                spine.set_visible(True)
            # 确保边框完全可见
            ax.patch.set_edgecolor('black')
            ax.patch.set_linewidth(3)
        
        # 创建按钮
        self.button1 = Button(self.button_ax1, 'Map1', 
                             color=self.colors['panel'], 
                             hovercolor=self.colors['optimal_path'])
        self.button2 = Button(self.button_ax2, 'Map2', 
                             color=self.colors['panel'], 
                             hovercolor=self.colors['optimal_path'])
        self.button3 = Button(self.button_ax3, 'Map3', 
                             color=self.colors['panel'], 
                             hovercolor=self.colors['optimal_path'])
        
        # 设置按钮文字样式
        for button in [self.button1, self.button2, self.button3]:
            button.label.set_color(self.colors['text'])
            button.label.set_fontweight('bold')
            button.label.set_fontsize(10)
        
        # 绑定点击事件
        self.button1.on_clicked(lambda x: self._on_map_button_click('map1.json'))
        self.button2.on_clicked(lambda x: self._on_map_button_click('map2.json'))
        self.button3.on_clicked(lambda x: self._on_map_button_click('map3.json'))
    
    def _on_map_button_click(self, map_file):
        """处理地图按钮点击事件"""
        if self.map_switch_callback:
            print(f"切换到地图: {map_file}")
            self.map_switch_callback(map_file)
    
    def _setup_data_panel(self):
        """设置数据栏"""
        self.data_ax.set_xlim(0, 1)
        self.data_ax.set_ylim(0, 1)
        self.data_ax.axis('off')
        self.data_ax.set_facecolor(self.colors['panel'])
        
        # 创建数据文本对象
        self.data_text = self.data_ax.text(0.05, 0.95, '', 
                                         ha='left', va='top',
                                         color=self.colors['text'],
                                         fontsize=10, fontweight='normal',
                                         transform=self.data_ax.transAxes)
        
        # 设置边框
        for spine in self.data_ax.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(2)
    
    def _setup_radar_panel(self):
        """设置雷达图"""
        self.radar_ax.set_xlim(-7, 7)
        self.radar_ax.set_ylim(-7, 7)
        self.radar_ax.set_facecolor(self.colors['background'])
        
        # 绘制雷达圆圈
        circles = [1, 2, 3, 4, 5, 6]
        for r in circles:
            circle = plt.Circle((0, 0), r, fill=False, color=self.colors['grid'], alpha=0.3, linewidth=0.5)
            self.radar_ax.add_patch(circle)
        
        # 绘制十字线
        self.radar_ax.axhline(y=0, color=self.colors['grid'], alpha=0.3, linewidth=0.5)
        self.radar_ax.axvline(x=0, color=self.colors['grid'], alpha=0.3, linewidth=0.5)
        
        # 添加距离标签
        for r in [2, 4, 6]:
            self.radar_ax.text(r, 0.2, f'{r}m', color=self.colors['text'], fontsize=8, ha='center')
        
        # 初始化雷达扫描线
        self.radar_lines = []
        self.radar_points = []
        
        # 设置边框
        for spine in self.radar_ax.spines.values():
            spine.set_edgecolor('black')
            spine.set_linewidth(2)

    def update(self, robot_pos, path=None, explore_map=None, optimal_path=None, visible_cells=None, slam_traj=None, slam_map=None, slam_pose=None, lidar_angles=None, lidar_distances=None):
        self._update_distance(robot_pos)
        if explore_map is not None:
            self.wall_img.set_data(explore_map)
        if path and len(path) > 1:
            px, py = zip(*path)
            self.path_line.set_data(py, px)
        else:
            self.path_line.set_data([], [])
        if optimal_path and len(optimal_path) > 1:
            opx, opy = zip(*optimal_path)
            self.optimal_line.set_data(opy, opx)
        else:
            self.optimal_line.set_data([], [])
        if robot_pos:
            self.robot_dot.set_data([robot_pos[1]], [robot_pos[0]])
        if visible_cells and len(visible_cells) > 0:
            vxs, vys = zip(*visible_cells)
            self.lidar_scatter.set_offsets(np.c_[vys, vxs])
        else:
            self.lidar_scatter.set_offsets(np.zeros((0,2)))
        if slam_traj and len(slam_traj) > 1:
            tx, ty = zip(*slam_traj)
            self.slam_path_line.set_data(ty, tx)
        else:
            self.slam_path_line.set_data([], [])
        if slam_map is not None:
            self.slam_img.set_data(slam_map)
        if self.slam_arrow is not None:
            self.slam_arrow.remove()
            self.slam_arrow = None
        if slam_pose is not None:
            (x, y), angle = slam_pose
            arrow_len = 6
            dx = arrow_len * np.cos(angle)
            dy = arrow_len * np.sin(angle)
            self.slam_arrow = self.slam_ax.arrow(
                y, x, dy, dx,
                width=1.5,
                color=self.colors['robot'],
                head_width=4,
                head_length=4,
                length_includes_head=True,
                zorder=10,
                alpha=0.8
            )

        # --------- 动态图例位置调整 START ----------
        def legend_loc_for_pos(pos, shape):
            if not pos:
                return 'upper right'
            x, y = pos
            h, w = shape
            # 右上1/4区域：x >= h // 2, y >= w // 2
            if x >= h // 2 and y >= w // 2:
                return 'lower right'
            return 'upper right'

        legend_loc = legend_loc_for_pos(robot_pos, self.wall_img.get_array().shape)
        if self.legend_handle is not None:
            self.legend_handle.remove()
            self.legend_handle = None
        self.legend_handle = self.ax.legend(fontsize=10, loc=legend_loc, frameon=True, fancybox=True, 
                                            shadow=True, framealpha=0.1, edgecolor=self.colors['text'])
        self.legend_handle.get_frame().set_facecolor(self.colors['panel'])
        for text in self.legend_handle.get_texts():
            text.set_color(self.colors['text'])

        slam_robot_pos = slam_pose[0] if slam_pose is not None else None
        slam_legend_loc = legend_loc_for_pos(slam_robot_pos, self.slam_img.get_array().shape)
        if self.slam_legend_handle is not None:
            self.slam_legend_handle.remove()
            self.slam_legend_handle = None
        self.slam_legend_handle = self.slam_ax.legend(fontsize=10, loc=slam_legend_loc, frameon=True, fancybox=True,
                                            shadow=True, framealpha=0.1, edgecolor=self.colors['text'])
        self.slam_legend_handle.get_frame().set_facecolor(self.colors['panel'])
        for text in self.slam_legend_handle.get_texts():
            text.set_color(self.colors['text'])
        # --------- 动态图例位置调整 END ----------
        
        self._update_info_panel(explore_map)
        
        # 更新数据栏
        self._update_data_panel(robot_pos, slam_pose, self._get_runtime())
        
        # 更新雷达图
        if lidar_angles is not None and lidar_distances is not None:
            self._update_radar_panel(lidar_angles, lidar_distances)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def _get_runtime(self):
        """获取运行时间"""
        return time.time() - self.start_time
    
    def _update_data_panel(self, robot_pos, slam_pose, runtime):
        """更新数据栏"""
        data_lines = []
        data_lines.append(f"Runtime: {runtime:.1f}s")
        data_lines.append(f"")
        data_lines.append(f"Robot Position:")
        data_lines.append(f"  X: {robot_pos[0]:.2f}")
        data_lines.append(f"  Y: {robot_pos[1]:.2f}")
        data_lines.append(f"")
        
        if slam_pose is not None:
            slam_x, slam_y = slam_pose[0]
            slam_theta = slam_pose[1]
            data_lines.append(f"SLAM Pose:")
            data_lines.append(f"  X: {slam_x:.2f}")
            data_lines.append(f"  Y: {slam_y:.2f}")
            data_lines.append(f"  θ: {np.degrees(slam_theta):.1f}°")
        else:
            data_lines.append(f"SLAM Pose: N/A")
        
        data_lines.append(f"")
        if hasattr(self, 'distance'):
            data_lines.append(f"Distance: {self.distance:.1f}")
        if hasattr(self, 'progress'):
            data_lines.append(f"Progress: {self.progress:.1f}%")
        
        data_text = "\n".join(data_lines)
        self.data_text.set_text(data_text)
    
    def _update_radar_panel(self, angles, distances):
        """更新雷达图"""
        # 清除之前的雷达线和点
        for line in self.radar_lines:
            line.remove()
        for point in self.radar_points:
            point.remove()
        self.radar_lines.clear()
        self.radar_points.clear()
        
        # 绘制新的雷达扫描
        for angle, distance in zip(angles, distances):
            # 计算终点坐标
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            # 绘制扫描线
            line, = self.radar_ax.plot([0, x], [0, y], 
                                     color=self.colors['radar_line'], 
                                     alpha=0.6, linewidth=0.8)
            self.radar_lines.append(line)
            
            # 绘制终点
            point, = self.radar_ax.plot(x, y, 'o', 
                                      color=self.colors['lidar'], 
                                      markersize=3, alpha=0.8)
            self.radar_points.append(point)
        
        # 绘制机器人位置（中心点）
        if not hasattr(self, 'radar_robot'):
            self.radar_robot, = self.radar_ax.plot(0, 0, 'o', 
                                                 color=self.colors['robot'], 
                                                 markersize=8, alpha=1.0)
        
        # 设置坐标轴
        self.radar_ax.set_xlim(-7, 7)
        self.radar_ax.set_ylim(-7, 7)
        self.radar_ax.tick_params(colors=self.colors['text'], labelsize=8)
    
    def _update_distance(self, robot_pos):
        """更新机器人行走距离"""
        if robot_pos and self.last_robot_pos:
            distance = np.sqrt((robot_pos[0] - self.last_robot_pos[0])**2 + 
                             (robot_pos[1] - self.last_robot_pos[1])**2)
            self.total_distance += distance
        if robot_pos:
            self.last_robot_pos = robot_pos
    
    def _calculate_exploration_progress(self, explore_map):
        """计算探索进度百分比"""
        if explore_map is None:
            return 0.0
        
        # 计算已探索的区域（非墙壁且非未知区域）
        total_cells = explore_map.size
        explored_cells = np.sum((explore_map == 0) | (explore_map == 2))  # 空地或已访问
        wall_cells = np.sum(explore_map == 1)  # 墙壁
        
        # 排除墙壁，计算可探索区域的探索百分比
        explorable_cells = total_cells - wall_cells
        if explorable_cells > 0:
            return (explored_cells / explorable_cells) * 100
        return 0.0
    
    def _get_runtime(self):
        """获取程序运行时间"""
        return time.time() - self.start_time
    
    def set_phase(self, phase):
        """设置当前阶段"""
        self.current_phase = phase
    
    def _update_info_panel(self, explore_map):
        """更新信息面板"""
        # 计算探索进度
        progress = self._calculate_exploration_progress(explore_map)
        
        # 创建更新的信息文本
        info_text = (f"Current Phase: {self.current_phase}  |  "
                    f"Runtime: {self._get_runtime():.1f}s  |  "
                    f"Distance: {self.total_distance:.2f}  |  "
                    f"Progress: {progress:.1f}%")
        
        # 直接更新文本内容，避免清除轴对象
        if hasattr(self, 'info_text'):
            self.info_text.set_text(info_text)
        else:
            self.info_text = self.info_ax.text(0.5, 0.5, info_text, 
                                              horizontalalignment='center', 
                                              verticalalignment='center',
                                              fontsize=12, color=self.colors['text'], 
                                              fontweight='bold',
                                              bbox=dict(boxstyle="round,pad=0.5", 
                                                       facecolor=self.colors['background'], 
                                                       edgecolor=self.colors['text'],
                                                       alpha=0.8))
