# costmap_prohibition_layer

#### 介绍
基于costmap_prohibition_layer [https://github.com/rst-tu-dortmund/costmap_prohibition_layer](https://github.com/rst-tu-dortmund/costmap_prohibition_layer) 
开源代码，魔改的支持文件配置动态虚拟墙，支持虚拟墙动态刷新

#### 软件架构
软件架构说明


#### 安装教程

1.  直接下载到你的工作空间上编译
2.  设置你的mve_base params文件(如果不知道在哪，百度学move_base调参)
3.  分别在global_costmap 和local_costmap 的文件中 plugins 下添加
    ’- {name: costmap_prohibition_layer,        type: "costmap_prohibition_layer_namespace::CostmapProhibitionLayer"}‘
4.  把spiritMaps.json文件放到主目录的map文件夹(如果没有就自己创建)下

#### 使用
1.  配置json文件修改保存
2.  rostopic 发布 /wall 参数任意 然后就动态更新了 

#### spiritMaps.json 说明

参数|类型|说明
--|:--:|--
shapeAreas|[]|存储所有虚拟墙的集合
shapeId|int64|虚拟墙id
type|string|虚拟墙类型（目前只支持lines）
closed|bool|是否闭合(首尾相接，组成形状，如：三角形，四边形等)
lines.x|double|虚拟墙坐标点x
lines.y|double|虚拟墙坐标点y

(由于该文件是从项目抽出，其他参数不做解析，大家用的时候，直接用文件默认就行)

