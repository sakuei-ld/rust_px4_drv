rust_px4_drv  
=======  

Rustで書いた px4_drv の移植ドライバ  
(USBデバイス向け)  
---
# 注意
現在の対応デバイスは PX-W3U4 のみ。  
対応OSは Mac OS のみ。  
最低限、rusb が動作すること。  
とりあえず、動作はするが、dropがどの程度抑えられているのかは不明(vs px4_drv)。  
デバッグ用に標準出力にログを大量に吐いている。

## インストール
rust で cargo build --release  
target/release にある実行ファイルを好きなところに配置。

## アプリ
rust-px4-drv-daemon: 常駐型の User Space Driver  
rust-px4-drv-shim: 上記 daemon の簡易クライアント  

## 使い方
- rust-px4-drv-daemon  
起動するだけ
- rust-px4-drv-shim  
コマンドは2種
  - 信号強度取得
  ```bash
  rust-px4-drv-shim signal <port> <chennel_type> <channel> [--lnb_on]
  ```
  - 録画
  ```bash
  rust-px4-drv-shim signal <port> <chennel_type> <channel> [output_filepath] [--lnb_on]
  ```
  引数は下記
  - port  
  チューナー番号  
  - channel_type  
  BS、CS、GR  
  - channel  
  15_0 とか 27 とか  
  - output_filepath  
  一応、無しも行けたはず。    
  標準出力は -

## ToDo
- (未定)Drop改善、リファクタ 2nd
- (未定)PX-Q3U4 対応
- (未定)PX-S1UR、PX-M1UR 対応
- (未定)Linux対応
- (未定)Windows対応

## その他
このアプリケーションは、下記を参考に実装しています。  
- px4_drv  
- recisdb-rs  

Gemini(無料版) および ChatGPT(無料版) を使い倒しました。