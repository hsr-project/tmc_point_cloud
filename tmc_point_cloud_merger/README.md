# tmc_point_cloud_merger

任意の数のsensor_msgs::msg::PointCloud2トピックを同一の基準座標に変換・合成し、ひとつのメッセージとして出版する

# 開発者

- 川田 福和
- 城 崇平

# I/F,Parameters

PKGDOC.rstに記載

# 使用方法

- 設定ファイルを用意する

  config/sample.yamlに説明あり

- 起動する
  下記コマンドで起動できる  
  $ ros2 launch tmc_point_cloud_merger sample.launch.py
