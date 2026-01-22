Overview
++++++++

提供機能
--------

複数の3次元点群データを、任意の基準Frameへ変換し、合成したうえで出版する

ROS Interface
+++++++++++++

Nodes
-----

- **point_cloud_merger**

  複数の3次元点群データを、任意の基準Frameへ変換し、合成したうえで出版するNode

Subscribed Topics
-----------------

設定次第で任意の数と名前の(:ros:msg:`sensor_msgs/msg/PointCloud2`)を購読する

fieldsにはx, y, zが含まれていることが求められる

rgb, rgbaは任意

Published Topics
^^^^^^^^^^^^^^^^

設定次第で任意の名前の合成した(:ros:msg:`sensor_msgs/msg/PointCloud2`)を発行する

マージ元のデータの色情報は失われる

Parameters
^^^^^^^^^^

配列で設定可能なパラメータもある。config/sample.yamlを参照

- **~inputs/(任意のinput名)/topic_name** (``string``)

  入力する(:ros:msg:`sensor_msgs/msg/PointCloud2`)のトピック名

- **~inputs/(任意のinput名)/stall_monitor_hz** (``double``)

  入力の受信途絶を監視するためのタイマーイベント発行周期(Hz)

- **~inputs/(任意のinput名)/stall_timeout_sec** (``double``)

  入力の受信が途切れたと断定するタイムアウト(sec)

- **~trigger/(任意のtrigger名)/topic_name** (``string``)

  合成した3次元点群データを発行するタイミングとして使用するトピック名

- **~output/topic_name** (``string``)

  合成した3次元点群データを出版する時のトピック名

- **~output/frame_id** (``string``)

  合成した3次元点群データの基準frame_id
