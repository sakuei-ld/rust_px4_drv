rust_px4_drv  
=======  

Rustで書いた px4_drv の移植ドライバ  
(USBデバイス向け)  
---
# 注意
現在の対応デバイスは PX-W3U4 のみ。  
動作確認OSは Mac のみ(TCP通信にしたので、どのOSでも動くと思いますが……)  
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
  - daemon server 起動
  ```
  rust-px4-drv-daemon [--host 0.0.0.0] [--port 40770] [--enable-bcas] [--bcas-proxy-port 6901]
  ```
  引数は下記
  - host  
  受ける ip アドレス。  
  default は 127.0.0.1 (ローカル限定)  
  0.0.0.0 を設定することで、ネットワークからのアクセスを許諾 (コンテナからのアクセスとか)
  - port  
  受ける port 番号。  
  default は 40771 (mirakc が 40772 だった気がするので)
  - enable-bcas  
  B-CAS の Proxy の有効化
  - bcas-proxy-port  
  B-CAS の Proxy サーバーのポート番号  
  default は 6900
- rust-px4-drv-shim  
コマンドは2種
  - 信号強度取得
  ```bash
  rust-px4-drv-shim signal <tuner_id> <chennel_type> <channel> [--lnb-on] [--host 192.168.1.10] [--port 40770] 
  ```
  - 録画
  ```bash
  rust-px4-drv-shim tune <tuner_id> <chennel_type> <channel> [output_filepath] [--lnb-on] [--host 192.168.1.10] [--port 40770] 
  ```
  引数は下記
  - tuner_id  
  チューナー番号  
  - channel_type  
  BS、CS、GR  
  - channel  
  15_0 とか 27 とか  
  - output_filepath  
  一応、無しも行けたはず。    
  標準出力は -
  - host  
  接続先 ipアドレス  
  - port  
  daemon の port番号

## ToDo
- (未定)Drop改善、リファクタ 2nd
- (未定)PX-Q3U4 対応
- (未定)PX-S1UR、PX-M1UR 対応

## その他
このアプリケーションは、下記を参考に実装しています。  
- px4_drv  
- recisdb-rs  

Gemini(無料版)、ChatGPT(無料版) および Claude(無料版) を使い倒しました。  
また、LocalLLM として、Qwen3.6 35B A3B および Qwen3.8 27B を試しています。
