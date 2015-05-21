#How to Install ROS in Ubuntu and make catkin_ws
It is still being edited.(編集中)   
ここでは、Ubuntu　15.04にROS　jadeをインストールし環境設定とワークスペースを作成するまでを解説します。

##参考
ROSインストール： 
http://wiki.ros.org/hydro/Installation/Ubuntu

環境設定とワークスペース作成： 
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


##公開鍵の登録
```
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```

##インストール
パッケージ・インデックスを最新の状態にする。
```
sudo apt-get update
```


すべてのデスクトップ環境をインストールする。その他の場合は、参考URLを参照。
```
sudo apt-get install ros-<ver.>-desktop-full
```
<ver.>は、jade（最新版）やhydroを入力
例）
sudo apt-get install ros-jade-desktop-full

利用可能なパッケージを検索するには
```
apt-cache search ros-<ver.>
```

##rosdepを初期化
ROSを使用する前に、rosdepを初期化する必要がある。
```
sudo rosdep init
rosdep update
```

##環境のセットアップ
ROSの環境変数が自動的にbashに読み込まれるようにするには
```
echo "source /opt/ros/<ver.>/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
とする。
その都度、自分で手入力する場合は、
```
source /opt/ros/<ver.>/setup.bash
```

##rosinstallの入手
```
sudo apt-get install python-rosinstall
```

##環境の管理
ROS文字列を検索し、大域変数として追加
```
export | grep ROS
```

```
$ source /opt/ros/<ver.>/setup.bash
```


##ROSのワークスペースを作成
catkin workspaceを作る。
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
これだけでは、ワークスペースは空の状態である。しかし、ビルドすることはできる。

```
cd ~/catkin_ws/
catkin_make
```

```
source devel/setup.bash
```

環境が正しくセットアップされているかをチェックする方法は
```
echo $ROS_PACKAGE_PATH
```
を入力し、
```
/home/youruser/catkin_ws/src:/opt/ros/<ver.>/share:/opt/ros/<ver.>/stacks
```
と出れば大丈夫。

