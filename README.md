# motor_driver
Cartographerによる2D Navigaionを行うロボットの製作
モーター制御部分のみ実装し、その他は各種パッケージを利用した

## 構成

### Launchファイル
ROSノードをまとめて起動するファイル

#### map_create.launch
手動操縦によるマップの作成用

- usb_cam_node
  - webカメラの表示だけ。不要ならコメントアウト
- rplidar
  - RPLiDARからのデータ取得とトピック配信を行うノード。
- cartographer_ros
  - lidarの情報をもとにSLAMを行うノード
- motor_driver
  - cmd_velを受けてarduinoへモーター速度指令を送るノード。ここだけ自作


#### move_base.launch
事前に作成したマップを使用する自動走行用

- usb_cam_node
  - webカメラの表示だけ。不要ならコメントアウト
- rplidar
  - RPLiDARからのデータ取得とトピック配信を行うノード。
- cartographer_ros
  - lidarの情報をもとにSLAMを行うノード
- move_base
  - マップと自己位置を元に行動計画と速度指令を送るノード。パラメータは少し調整した
- motor_driver
  - cmd_velを受けてarduinoへモーター速度指令を送るノード。ここだけ自作


## 環境構築
### RPLiDAR のインストール
[ここ](https://github.com/Slamtec/rplidar_ros)からクローンしてビルド
(ソースを取ってこなくても、aptでパッケージをインストールできるかも？)
```
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar
catkin build --this # もしくはcatkin_make

```
### Cartographer のインストール
- [Compiling Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#system-requirements)

- [RaspberryでROSのCartographerを使用してVNCでホストコンピューターからrviz見たくてめちゃくちゃハマった人（人）](https://qiita.com/ink_off/items/11ca70f411206f4e7aa6)


### move_base のインストール
```
sudo apt-get install -y ros-melodic-move-base
```
[ROS講座92 move_baseで移動する](https://qiita.com/srs/items/8e65f04976abb837d207) 
ROS全般はこのサイトが詳しいので、困ったら該当する記事を参考にする。というか検索すると必ずこのサイトにたどり着く

### OSインストール
ROSインストール済のUbuntuイメージを入手してSDカードに焼く
b8:27:eb:8a:30:a7
- [Raspberry Pi 3B+用Ubuntu 18.04+ROSイメージ](https://b.ueda.tech/?post=20190618_raspimouse)


- 同記事の[有線LANの設定]の手順を行う
- ついでに無線LANの設定も行う

- [Ubuntu Server 18.04 LTSで無線LANとsshの設定をするまで](https://qiita.com/borchi-no/items/69e8c3cea884da111c95)

↑の記事だけだとつながらない。↓をやる

- [ラズパイ3で無線LANを有効にした時のお話](https://ochanjanai.net/it/146)

```
cd /lib/firmware/brcm/
wget https://github.com/RPi-Distro/firmware-nonfree/raw/master/brcm/brcmfmac43430-sdio.bin
wget https://github.com/RPi-Distro/firmware-nonfree/raw/master/brcm/brcmfmac43430-sdio.txt
shutdown -r now
```

```
wifis:
            インターフェイス名:
                    dhcp4: true
                    access-points:
                        "SSID名":
                                 password: "パスワード"
```

```
sudo apt install network-manager
sudo netplan apply
```


## スワップファイルの作成
Raspi3B+のメモリ1GBではcatkinビルドする時にメモリ不足でエラーになるので、
スワップ領域を作成する

```
# スワップファイルを作成する。1GBでは足りなかったので4GB
sudo fallocate -l 4G /swapfile

# rootしか読み書きできない権限にする
sudo chmod 600 /swapfile

# ファイルにスワップ領域を作成する
sudo mkswap /swapfile

# スワップを有効にする
sudo swapon /swapfile

# 有効になっていることを確認する
sudo swapon --show
# NAME    TYPE    SIZE    USED    PRIO
# /swapfile file 4096M 0B

# 起動時に自動マウントされるように設定する
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
# /swapfile swap swap defaults 0 0

# 再起動して確認する
sudo reboot

sudo swapon --show
# NAME      TYPE  SIZE USED PRIO
# /swapfile file 1024M   0B   -2

```

- [Raspberry PiのUbuntuにスワップ領域を設定する](https://qiita.com/zrock/items/71e3874cb83ed12ec405)
