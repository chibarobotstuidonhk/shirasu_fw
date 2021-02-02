

# パラメータ
```
shell>parameter
```
で今の"parameter"の値を取得
```
shell>parameter value
```
で"paramter"を"value"に設定する.
```
shell>WCFG
```
で今のパラメータをフラッシュに書き込む.
次回以降の起動時にフラッシュからパラメータを読み込む.
フラッシュに書き込まれる(読み込まれる)のは以下のパラメータ
```
KPR
KIT
KVP
DEF
BID
CPR(PPR)
```
"parameter"は下のいずれか
## KPR
速度比例ゲイン[A/(rad/s)]
## KIT
速度積分ゲイン[A/(rad/s)]
## KVP
位置比例ゲイン[(rad/s)/rad]
## CPR
回転あたりのカウント[Counts per Revolution]
## PPR
回転あたりのパルス[Pulses per Revolution]
CPR = PPR*4の関係がある
したがってCPRかPPRのいずれかを設定するだけでいい
## MODE
モード
ただしdisable以外のモードは非常停止が押されていない時のみ可能.
| MODE |モード|
| ---- | ---- |
|  DEF |デフォルトモード|
|  CUR |電流制御モード|
|  VEL  |速度制御モード|
|  POS  |位置制御モード|
|  それ以外  |disable|
## DEF
デフォルトのモード
後方互換性のために用意
| DEF |デフォルトモード|
| ---- | ---- |
|  CUR |電流制御モード|
|  VEL  |速度制御モード|
|  POS  |位置制御モード|
|  それ以外  |disable|
## TARGET
modeによって挙動が変わる.
| モード |TARGET|
| ---- | ---- |
|  disable  |無視|
|  電流制御モード  |目標電流[A]|
|  速度制御モード  |目標速度[rad/s]|
|  位置制御モード  |目標位置[rad]|
## BID
CAN BUS ID
0x000 ~ 0x00Cは緊急停止などのために予約.
0x00C ~ 0x7ffで4の倍数が設定出来る.
一つのMDにつきBID,BID+1,BID+2,BID+3のIDを使用する.
### BID(cmd)
uint8_tのデータが送れる.
dataによってMDのモードが遷移する.
ただしdisable以外のモードは非常停止が押されていない時のみ可能.
| data | mode |
| ---- | ---- |
|  0   |  disable  |
|  1   |  default  |
|  2   |  current  |
|  3   |  velocity  |
|  4   |  position  |
### BID+1(target)
floatのデータが送れる.
modeによって挙動が変わる.
| mode |target|
| ---- | ---- |
|  disable  |無視|
|  current  |目標電流[A]|
|  velocity  |目標速度[rad/s]|
|  position  |目標位置[rad]|
### BID+2
予約
### BID+3
予約

## TEMP (readonly)
温度[degree]
## VSP (readonly)
電源電圧[V]
## 例
パルス数を変えた場合はPPR(CPR)をいじればいい.
エンコーダの正方向とモーターの正方向が違うときはPPR(CPR)を-1倍する.
ギヤ比を変えた場合はKPR,KITを前のギヤ比/後のギヤ比倍すればいい(あくまで目安).
### 775 24:1 500パルス
KPR 2.6
KIT 100
KVP 30
PPR -500


# 性能
| 運転条件 |最小|最大|
| ---- | ---- | ---- |
|  電源電圧[V]  |12|24|
|  電流(瞬間)[A]  ||60|