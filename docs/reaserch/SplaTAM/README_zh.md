<!-- PROJECT LOGO -->

<p align="center">

  <h1 align="center">SplaTAM：基于3D高斯的稠密RGB-D SLAM建图与追踪</h1>
  <h3 align="center">CVPR 2024</h3>
  <p align="center">
    <a href="https://nik-v9.github.io/"><strong>Nikhil Keetha</strong></a>
    ·
    <a href="https://jaykarhade.github.io/"><strong>Jay Karhade</strong></a>
    ·
    <a href="https://krrish94.github.io/"><strong>Krishna Murthy Jatavallabhula</strong></a>
    ·
    <a href="https://gengshan-y.github.io/"><strong>Gengshan Yang</strong></a>
    ·
    <a href="https://theairlab.org/team/sebastian/"><strong>Sebastian Scherer</strong></a>
    <br>
    <a href="https://www.cs.cmu.edu/~deva/"><strong>Deva Ramanan</strong></a>
    ·
    <a href="https://www.vision.rwth-aachen.de/person/216/"><strong>Jonathon Luiten</strong></a>
  </p>
  <h3 align="center"><a href="https://arxiv.org/pdf/2312.02126.pdf">论文</a> | <a href="https://youtu.be/jWLI-OFp3qU">视频</a> | <a href="https://spla-tam.github.io/">项目主页</a></h3>
  <div align="center"></div>
</p>

<p align="center">
  <a href="">
    <img src="./assets/1.gif" alt="Logo" width="100%">
  </a>
</p>

<br>

## 敬请期待更快更好的SplaTAM版本！

<!-- TABLE OF CONTENTS -->
<details open="open" style='padding: 10px; border-radius:5px 30px 30px 5px; border-style: solid; border-width: 1px;'>
  <summary>目录</summary>
  <ol>
    <li>
      <a href="#安装">安装</a>
    </li>
    <li>
      <a href="#在线演示">在线演示</a>
    </li>
    <li>
      <a href="#使用方法">使用方法</a>
    </li>
    <li>
      <a href="#数据下载">数据下载</a>
    </li>
    <li>
      <a href="#基准测试">基准测试</a>
    </li>
    <li>
      <a href="#致谢">致谢</a>
    </li>
    <li>
      <a href="#引用">引用</a>
    </li>
    <li>
      <a href="#开发者">开发者</a>
    </li>
  </ol>
</details>

## 安装

##### （推荐）
SplaTAM已在Python 3.10、Torch 1.12.1和CUDA 11.6环境下进行了基准测试。然而，Torch 1.12并非硬性要求，代码也在其他版本的Torch和CUDA（如Torch 2.3.0和CUDA 12.1）上进行了测试。

安装所有依赖项最简单的方法是使用[anaconda](https://www.anaconda.com/)和[pip](https://pypi.org/project/pip/)，步骤如下：

```bash
conda create -n splatam python=3.10
conda activate splatam
conda install -c "nvidia/label/cuda-11.6.0" cuda-toolkit
conda install pytorch==1.12.1 torchvision==0.13.1 torchaudio==0.12.1 cudatoolkit=11.6 -c pytorch -c conda-forge
pip install -r requirements.txt
```

<!-- 另外，我们还提供了conda环境配置文件：
```bash
conda env create -f environment.yml
conda activate splatam
``` -->

#### Windows

在Windows上使用Git bash安装，请参考[Issue#9中分享的说明](https://github.com/spla-tam/SplaTAM/issues/9#issuecomment-1848348403)。

#### Docker和Singularity设置

我们也提供了Docker镜像。我们建议在Docker镜像内使用虚拟环境运行代码：

```bash
docker pull nkeetha/splatam:v1
bash bash_scripts/start_docker.bash
cd /SplaTAM/
pip install virtualenv --user
mkdir venv
cd venv
virtualenv --system-site-packages splatam
source ./splatam/bin/activate
pip install -r venv_requirements.txt
```

设置Singularity容器类似：
```bash
cd </path/to/singularity/folder/>
singularity pull splatam.sif docker://nkeetha/splatam:v1
singularity instance start --nv splatam.sif splatam
singularity run --nv instance://splatam
cd <path/to/SplaTAM/>
pip install virtualenv --user
mkdir venv
cd venv
virtualenv --system-site-packages splatam
source ./splatam/bin/activate
pip install -r venv_requirements.txt
```

## 演示

### 在线演示

您可以通过下载并使用<a href="https://apps.apple.com/au/app/nerfcapture/id6446518379">NeRFCapture</a>应用，使用iPhone或配备LiDAR的Apple设备对您的环境进行SplaTAM建图。

确保您的iPhone和PC连接到同一个WiFi网络，然后运行以下命令：

```bash
bash bash_scripts/online_demo.bash configs/iphone/online_demo.py
```

在应用程序中，持续点击发送以获取连续帧。捕获完成后，应用程序将断开与PC的连接，您可以在PC上查看SplaTAM重建的交互式渲染效果！以下是一些酷炫的示例结果：

<p align="center">
  <a href="">
    <img src="./assets/collage.gif" alt="Logo" width="75%">
  </a>
</p>

### 离线演示

您也可以先捕获数据集，然后使用以下命令对数据集运行SplaTAM：

```bash
bash bash_scripts/nerfcapture.bash configs/iphone/nerfcapture.py
```

### 数据集捕获

如果您只想使用NeRFCapture应用程序捕获自己的iPhone数据集，请使用以下命令：

```bash
bash bash_scripts/nerfcapture2dataset.bash configs/iphone/dataset.py
```

## 使用方法

我们将以iPhone数据集为例，展示如何使用SplaTAM。对于其他数据集，步骤类似。

运行SplaTAM，请使用以下命令：

```bash
python scripts/splatam.py configs/iphone/splatam.py
```

要可视化最终的交互式SplaTAM重建结果，请使用以下命令：

```bash
python viz_scripts/final_recon.py configs/iphone/splatam.py
```

要在线可视化SplaTAM重建过程，请使用以下命令：

```bash
python viz_scripts/online_recon.py configs/iphone/splatam.py
```

要将高斯数据导出为.ply文件，请使用以下命令：

```bash
python scripts/export_ply.py configs/iphone/splatam.py
```

`PLY`格式的高斯数据可以在[SuperSplat](https://playcanvas.com/supersplat/editor)和[PolyCam](https://poly.cam/tools/gaussian-splatting)等查看器中可视化。

要在SplaTAM重建结果上运行3D高斯切片，请使用以下命令：

```bash
python scripts/post_splatam_opt.py configs/iphone/post_splatam_opt.py
```

要使用真实位姿在数据集上运行3D高斯切片，请使用以下命令：

```bash
python scripts/gaussian_splatting.py configs/iphone/gaussian_splatting.py
```

## 数据下载

DATAROOT默认为`./data`。如果数据集存储在计算机的其他位置，请更改特定场景配置文件中的`input_folder`路径。

### Replica

按以下命令下载数据，数据将保存到`./data/Replica`文件夹。请注意，Replica数据由iMAP作者生成（但由NICE-SLAM作者托管）。如果使用数据，请引用iMAP。

```bash
bash bash_scripts/download_replica.sh
```

### TUM-RGBD

```bash
bash bash_scripts/download_tum.sh
```

### ScanNet

请按照[ScanNet](http://www.scan-net.org/)网站上的数据下载流程操作，并使用[此代码](https://github.com/ScanNet/ScanNet/blob/master/SensReader/python/reader.py)从`.sens`文件中提取彩色/深度帧。

<details>
  <summary>[ScanNet目录结构（点击展开）]</summary>

```
  DATAROOT
  └── scannet
        └── scene0000_00
            └── frames
                ├── color
                │   ├── 0.jpg
                │   ├── 1.jpg
                │   ├── ...
                │   └── ...
                ├── depth
                │   ├── 0.png
                │   ├── 1.png
                │   ├── ...
                │   └── ...
                ├── intrinsic
                └── pose
                    ├── 0.txt
                    ├── 1.txt
                    ├── ...
                    └── ...
```
</details>

我们使用以下序列：
```
scene0000_00
scene0059_00
scene0106_00
scene0181_00
scene0207_00
```

### ScanNet++

请按照<a href="https://kaldir.vc.in.tum.de/scannetpp/">ScanNet++</a>网站上的数据下载和图像去畸变流程操作。

另外，为了去畸变DSLR深度图像，我们使用了<a href="https://github.com/Nik-V9/scannetpp">官方ScanNet++处理代码的修改版</a>。我们很快会向官方ScanNet++仓库提交拉取请求。

我们使用以下序列：

```
8b5caf3398
b20a261fdf
```

对于b20a261fdf，我们使用前360帧，因为第360帧后轨迹会出现突变/跳转。请注意，ScanNet++主要设计为NeRF训练和新视角合成数据集。

### Replica-V2

我们使用vMAP的Replica-V2数据集来评估新视角合成。请从<a href="https://github.com/kxhit/vMAP">vMAP</a>下载预生成的Replica序列。

## 基准测试

运行SplaTAM时，我们推荐使用[weights and biases](https://wandb.ai/)进行日志记录。可以通过在配置文件中设置`wandb`标志为True来启用。同时确保指定`wandb_folder`路径。如果您没有wandb账户，请先创建一个。请确保将`entity`配置更改为您的wandb账户。每个场景都有一个配置文件夹，需要在其中指定`input_folder`和`output`路径。

以下我们展示每个数据集一个场景的示例运行命令。SLAM运行后，将评估轨迹误差和渲染指标。结果默认保存到`./experiments`目录。

### Replica

要在`room0`场景上运行SplaTAM，请运行以下命令：

```bash
python scripts/splatam.py configs/replica/splatam.py
```

要在`room0`场景上运行SplaTAM-S，请运行以下命令：

```bash
python scripts/splatam.py configs/replica/splatam_s.py
```

对于其他场景，请修改`configs/replica/splatam.py`文件或使用`configs/replica/replica.bash`。

### TUM-RGBD

要在`freiburg1_desk`场景上运行SplaTAM，请运行以下命令：

```bash
python scripts/splatam.py configs/tum/splatam.py
```

对于其他场景，请修改`configs/tum/splatam.py`文件或使用`configs/tum/tum.bash`。

### ScanNet

要在`scene0000_00`场景上运行SplaTAM，请运行以下命令：

```bash
python scripts/splatam.py configs/scannet/splatam.py
```

对于其他场景，请修改`configs/scannet/splatam.py`文件或使用`configs/scannet/scannet.bash`。

### ScanNet++

要在`8b5caf3398`场景上运行SplaTAM，请运行以下命令：

```bash
python scripts/splatam.py configs/scannetpp/splatam.py
```

要在`8b5caf3398`场景上运行新视角合成，请运行以下命令：

```bash
python scripts/eval_novel_view.py configs/scannetpp/eval_novel_view.py
```

对于其他场景，请修改`configs/scannetpp/splatam.py`文件或使用`configs/scannetpp/scannetpp.bash`。

### ReplicaV2

要在`room0`场景上运行SplaTAM，请运行以下命令：

```bash
python scripts/splatam.py configs/replica_v2/splatam.py
```

要在SplaTAM运行后在`room0`场景上运行新视角合成，请运行以下命令：

```bash
python scripts/eval_novel_view.py configs/replica_v2/eval_novel_view.py
```

对于其他场景，请修改配置文件。

## 致谢

我们感谢以下开源代码仓库的作者：

- 3D高斯
  - [Dynamic 3D Gaussians](https://github.com/JonathonLuiten/Dynamic3DGaussians)
  - [3D Gaussian Splating](https://github.com/graphdeco-inria/gaussian-splatting)
- 数据加载器
  - [GradSLAM & ConceptFusion](https://github.com/gradslam/gradslam/tree/conceptfusion)
- 基线方法
  - [Nice-SLAM](https://github.com/cvg/nice-slam)
  - [Point-SLAM](https://github.com/eriksandstroem/Point-SLAM)

## 引用

如果您觉得我们的论文和代码有用，请引用我们：

```bib
@inproceedings{keetha2024splatam,
        title={SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM},
        author={Keetha, Nikhil and Karhade, Jay and Jatavallabhula, Krishna Murthy and Yang, Gengshan and Scherer, Sebastian and Ramanan, Deva and Luiten, Jonathon},
        booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
        year={2024}
      }
```

## 开发者
- [Nik-V9](https://github.com/Nik-V9) ([Nikhil Keetha](https://nik-v9.github.io/))
- [JayKarhade](https://github.com/JayKarhade) ([Jay Karhade](https://jaykarhade.github.io/))
- [JonathonLuiten](https://github.com/JonathonLuiten) ([Jonathan Luiten](https://www.vision.rwth-aachen.de/person/216/))
- [krrish94](https://github.com/krrish94) ([Krishna Murthy Jatavallabhula](https://krrish94.github.io/))
- [gengshan-y](https://github.com/gengshan-y) ([Gengshan Yang](https://gengshan-y.github.io/))