# 训练参数和 loss\PR 曲线

## 数据扩增处理

原来图像训练集在 `/train_yolo11/yolo_dataset/train` 文件夹下，原训练集对应的 `images` 和 `labels` 分别有 106 张图片和标签

加载足球场场景，并调整相机视角保存 20 张新增图片

在默认保存位置 `/home/.gazebo/pictures` 重命名图片，序号保持递增，并命名规则延续 107-126，方便加入训练集和测试集

`labelImg` 默认保存格式是 `xml`，在UI界面切换到YOLO格式

标签保存时会默认生成一个 `classes.txt` 文件，转移到训练集的时候删除

0: Ball (球)

1: Post (门柱)

2: L (L形标记)

3: T (T形标记)

4: X (X形标记)

第一次标记时按照上述顺序标记，方便 `classes.txt` 对于序号顺序处理
标记完成后，将 107-121 的序号样本加入训练集，将 122-126 的集合加入测试集

## 扩增后训练参数修改

- 仅修改参数 `batch` 和 `lr0`，其他参数保持不变

| 参数名       | 修改前取值             | 修改后取值             |
| ------------ | ---------------------- | ---------------------- |
| data         | `config_path`          | `config_path`          |
| epochs       | `1050`                 | `1050`                 |
| batch        | `16`                   | `32`                   |
| imgsz        | `640` (=320×2)         | `640` (=320×2)         |
| device       | `'0'`                  | `'0'`                  |
| workers      | `8`                    | `8`                    |
| optimizer    | `'auto'`               | `'auto'`               |
| project      | `'./runs/train'`       | `'./runs/train'`       |
| name         | `'football_detection'` | `'football_detection'` |
| exist\_ok    | `True`                 | `True`                 |
| augment      | `True`                 | `True`                 |
| lr0          | `0.005`                | `0.003`                |
| lrf          | 0.2                    | `0.2`                  |
| save\_period | 10                     | `10`                   |
| amp          | True                   | `True`                 |
| patience     | 100                    | `100`                  |

## 修改前模型性能

### loss 曲线

![results](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/results.png)

### PR 曲线

![PR_curve](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/PR_curve.png)

## 修改后模型性能

### loss 曲线

![results](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/results.png)

### PR 曲线

![PR_curve](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/PR_curve.png)
