# Python 包的配置

## 引言

在 Python 中，模块（module）和包（package）是组织代码的两种结构：

1. **模块**：一个 `.py` 文件，例如 `math_utils.py`
2. **包**：包含 `__init__.py` 的文件夹，例如 `my_utils/`

在 Python 中，多个文件夹（包）之间的相互调用，主要依赖于**模块导入机制**和**包结构设计**

- 模块导入机制简介

1. **相对导入**：适用于包内部模块之间的调用，如 `from .import xxx`。
2. **绝对导入**：适用于项目中不同文件夹（包）之间的调用，如 `from package.module import xxx`

- 包结构设计简介

下面是一个python项目的常见结构，其中 `main.py` 是程序入口：

```bash
my_project/
├── main.py
├── utils/
│   └── math_utils.py
├── models/
│   └── model.py

```

Python 中 `__init__.py` 是一个包的“初始化文件”，可以根据项目复杂度灵活配置

## 模块导入方法和导入机制

### 绝对导入（Absolute Import）

- 形式

```python
from package.submodule import func
import package.module
```

- 特点

1. 从项目的根目录（顶级包）出发
2. 最清晰、最推荐
3. 可读性强，跨文件夹导入容易

- 使用条件示例

项目根目录（`project/`）必须在 `sys.path` 或当前工作目录下，结构示意如

```bash
project/ # 当前工作目录
├── main.py
├── my_utils/
│   ├── __init__.py
│   └── math_utils.py
```

### 相对导入（Relative Import）

- 形式

```python
from . import math_utils        # 当前目录
from .. import my_utils         # 上一级目录
from ..filters import butter    # 上一级的 filters 子模块
```

- 特点

1. 从当前模块的位置出发
2. 必须在 包中 使用，不能在脚本入口（`__main__`）里用
3. 适用于包内部子模块间调用

- 使用条件示例

在结构如：

```bash
my_package/
├── __init__.py
├── math/
│   ├── __init__.py
│   └── algebra.py
└── utils/
    ├── __init__.py
    └── tools.py
```

如果想在 `utils/` 文件下的 `tools.py` 中调用 `algebra` 函数，写：

```python
from ..math import algebra
```

❗ **注意**：不能直接运行 `tools.py`，否则报错

### 导入机制

Python 的导入遵循以下顺序查找模块路径：

```python
import sys
print(sys.path)
```

其返回的是一个路径列表，如

```python
[
  '',  # 当前运行脚本所在目录
  '/usr/lib/python3.10',
  '/usr/lib/python3.10/lib-dynload',
  '/home/bohao/.local/lib/python3.10/site-packages',
  ...
]
```

顺序是

1. 当前脚本所在目录
2. `PYTHONPATH` 环境变量中的路径，若配置此**仅当前 shell 有效**
3. 安装的标准库路径（site-packages），**永久有效**

还有就是动态添加路径，例如

```python
import sys
sys.path.append('/home/bohao/my_project')
```

这种等价于我们在 `MATLAB` 常用的 `addpath` 方式，**仅当前运行有效**

- 使用标准规则

| 场景                | 建议导入方式               |
| ------------------- | -------------------------- |
| 项目模块之间        | 使用绝对导入（推荐）       |
| 包内多个子模块之间  | 可以用相对导入（清晰）     |
| 脚本入口（main.py） | 只能用绝对导入             |
| 跨包调用            | 用绝对路径 + 配置 sys.path |

## 包结构设计

文件结构示例：

```bash
my_utils/
├── __init__.py
├── math_tools.py        # 提供 add(), subtract()
└── filters.py            # 提供 low_pass_filter()
```

主要是配置`__init__.py` 函数，告诉 Python 这是一个包（Python  虽然可以省略，但建议保留）

```python
# my_utils/__init__.py
# 空文件，也可以什么都不写
```

这就足以在外部使用：

```python
from my_utils import my_module
```

### 进阶使用 1-对外暴露包的核心函数/类

可以在 `__init__.py` 中导入包中常用的内容，让使用更简洁：

```python
# my_utils/__init__.py
from .math_tools import add, subtract
from .filters import low_pass_filter

__all__ = ['add', 'subtract', 'low_pass_filter'] #用于控制“通配导入（wildcard import）”行为的变量声明
```

这样外部就可以这么写，而不用知道内部模块结构：

```python
from my_utils import add, low_pass_filter
```

类似于 MATLAB 的：

```matlab
addpath('my_utils')
add(1,2)  % 不用管 add 是在哪个 .m 里
```

特别注意， `__all__` 它告诉 Python：当你使用 `from xxx import \*` 时，哪些名字可以被导入，只有 `__all__` 中列出的名字会被导入。这是对模块接口的一种显式封装。没有 `__all__` ，默认会导入所有**不以下划线开头**的变量/函数/类（可能暴露太多内部实现）

### 进阶使用 2-设置默认参数、常量、路径

```python
# my_utils/__init__.py
import os

MY_UTILS_ROOT = os.path.dirname(os.path.abspath(__file__)) # 当前绝对路径的父路径
DEFAULT_CONFIG = {
    'filter_order': 2,
    'sample_rate': 1000,
}
```

可以在包的其他地方用：

```python
from my_utils import DEFAULT_CONFIG
```

这类似于 MATLAB 在 `startup.m` 中设置全局变量。

### 进阶使用 3-初始化日志、打印进入信息

```python
# my_utils/__init__.py
print("[my_utils] 包已成功加载！")

# 或者初始化日志系统
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
```

### 进阶使用 4-封装内部模块（子模块初始化）

如果 `my_utils` 目录下有很多子模块，比如：

```
my_utils/
├── __init__.py
├── math_tools.py
├── string_tools.py
├── filters/
│   ├── __init__.py
│   └── butterworth.py
```

可以在 `my_utils/__init__.py` 中导入子包内容：

```python
from .math_tools import *
from .string_tools import *
from .filters import butterworth
```

这样一行 `from my_utils import butterworth` 就能访问深层模块（这里是相对导入）

### 使用 `__all__` 控制导入行为

例如

```python
# my_utils/__init__.py
__all__ = ['add', 'low_pass_filter']
```

这样当你使用：

```python
from my_utils import *
```

只会导入这两个符号相关的函数或类
