# planning-car
Some common planning algorithms for car

map_server -> /map
配置文件里分辨率不要动，保持为1

plan_env: 地图管理，提供操作地图的API

a_star: A*算法

运行：

    rosluanch a_star astar_demo.launch

使用2D Nav Goal点击地图即可，第一次是起点，第二次是终点

hybrid a star
map中每个栅格不只取中心，而是包含很多状态($x,y,\phi$)，因此，每个大栅格会离散成一个三维($x,y,\phi$)栅格地图，拥有更精细的分辨率，grid_size_xy_和grid_size_phi_分别描述了每个大栅格中有几个小xy栅格和整体$\phi$维度栅格的个数