# planning-car
Some common planning algorithms for car

map_server -> /map
配置文件里分辨率不要动，保持为1

plan_env: 地图管理，提供操作地图的API

a_star: A*算法

运行：

    rosluanch a_star astar_demo.launch

使用2D Nav Goal点击地图即可，第一次是起点，第二次是终点