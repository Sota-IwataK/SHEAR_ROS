#CubeSLAMで取得した直方体の情報をpublishするためのカスタムメッセージ
int8 id #ここはオブジェクトの名前に変えなければいけい（今は対象物が一つなのでこのままでいい）
int8 corner_num #頂点の番号（あらかじめ決まっている）
geometry_msgs/Pose posi #頂点の座標