# Test Piece ROS2

このプロジェクトは、ROS2を使用して画像を取得し、画像処理を行うためのパイプラインを提供します。画像を取得するためのパブリッシャーと、処理された画像を受信するためのサブスクライバーが含まれています。

## 必要条件

- ROS2（Foxyまたはそれ以降のバージョン）
- OpenCV
- cv_bridge

## セットアップ

プロジェクトをビルドするには、次のコマンドを実行してください。

```bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
