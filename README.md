**概要**
---
このパッケージは、RoboCup@Home OPL JapanOpen2019専用タスク「Go get in unknown environment(以下GoGetIt)」のタスク設計に関するパッケージである。
- ggi_master.py:状態遷移管理プログラム
- ggi_navigation.py:ナビゲーション用プログラム

ggi_master.py
---
各フェーズ（Traning Phase,Test Phase）の状態遷移に関するプログラムである。

ggi_navigation.py
---
ggi_master.pyで利用するためのナビゲーションに関する専門のプログラムである。ここでは、場所の記憶・場所への移動といったタスクを行う。
