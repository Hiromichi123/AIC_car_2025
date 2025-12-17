SESSION = my_grid_env

launch:
	# 1. 创建会话 (左上角 - Pane 0)
	tmux new-session -d -s $(SESSION) -n "Robot Real"
	tmux send-keys -t $(SESSION) "ros2 launch robot_real all.launch.py" C-m

	# 2. 水平切分屏幕 (此时变为左右两半，光标在右边 - Pane 1)
	tmux split-window -h -t $(SESSION)
	tmux send-keys -t $(SESSION) "ros2 run yolip yolip" C-m

	# 3. 垂直切分右边的屏幕 (此时右边变为上下两半，光标在右下 - Pane 2)
	tmux split-window -v -t $(SESSION)
	tmux send-keys -t $(SESSION) "ros2 run tts tts_node" C-m

	# 4. 选择左边的屏幕 (Pane 0) 并垂直切分 (此时左边变为上下两半 - 新 Pane)
	tmux select-pane -t $(SESSION):0.0
	tmux split-window -v -t $(SESSION)
	tmux send-keys -t $(SESSION) "sleep 20; ros2 run navi_rs navi_rs" C-m

	# 调整布局为平铺 (Tiled) 确保大小均匀 (可选)
	tmux select-layout -t $(SESSION) tiled

	# 进入会话
	tmux attach-session -t $(SESSION)

stop:
	tmux kill-session -t $(SESSION)

.PHONY: build
build:
	colcon build --packages-select ros2_tools robot_real vision_node tts yolip navi_rs

.PHONY: clean
clean:
	rm -rf build install log