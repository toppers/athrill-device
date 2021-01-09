# athrill-device
athrill device for external shared library.

## athrill-device 紹介記事
* [仮想 IoT デバイスを Athrill で作る！](https://qiita.com/kanetugu2018/items/5bf890c0945c299eb7f0)

## 開発言語
athrill-device は、基本的に C/C++ で開発することを想定しています。一方で、athrill-device は、様々な開発者に利用していただけるように、多言語対応したいと考えています。そのための対応として、[grpc](https://grpc.io) との連携を行えるようにしました。

具体的には、athrill-device 側から grpc のクライアント RPC API(C++) を呼び出し、サーバー側のプログラミング言語でデバイス開発を行います。サーバー側のプログラミング言語は、例えば以下が[サポート対象の言語](https://www.grpc.io/docs/languages/)となっています。

* C++
* C#
* Dart
* Go
* Java
* Kotlin/JVM
* Node
* Objective-C
* PHP
* Python
* Ruby
  
この他にも、[gRPC の GitHub Organization](https://github.com/grpc) では、Web Clients や Haskell などが公開されています。
また、コミュニティベースとして [Elixir](https://github.com/elixir-grpc/grpc) などのプログラミング言語もサポートされています。

## grpc を利用したサンプルデバイス
grpc を用いた単純なデバイス例として、Ruby と Elixir による [sampleデバイス](https://github.com/toppers/athrill-device/tree/main/device/sample) を公開しています。

ここでは、デバイスのクロック情報を grpc の RPC API の引数で渡して、サーバー側(Ruby)プログラムで参照・デバッグ出力する例を説明します。（Elixir によるサンプルは[こちら](https://github.com/toppers/athrill-device/tree/main/device/sample/grpc/elixir)を参照してください）

* RPC proto
  * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/proto/sample.proto
* クライアント側
  * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/client/cpp/sample_client.cc
* サーバー側
  * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/server/ruby/sample_server.rb

grpc のインストールは若干面倒なので、Dockerファイルでインストール方法を公開しました。

https://github.com/toppers/athrill-device/blob/main/docker/v850/Dockerfile.athrill

以下の手順でインストールおよび動作確認できます。

* まず、以下のリポジトリをクローンします。

```
$ cd <work_dir※作業フォルダを作ってください>
$ git clone https://github.com/toppers/athrill.git
$ git clone https://github.com/toppers/athrill-target-v850e2m.git
$ git clone https://github.com/toppers/athrill-device.git
```

* athrill-device 上で docker イメージを作成します。

```
$ cd athrill-device
$ cd docker/v850
$ bash create-image-athrill.bash
```

* docker コンテナを起動します。

```
$ bash run-athrill.bash 
```

* docker コンテナ上で athrill, サンプルデバイスをビルド・インストールします。

```
# bash install-athrill.bash
# cd athrill-device/device/sample/build
# bash build.bash all
```

* 別の端末から docker コンテナに入り、Ruby 側のサーバーを起動します。

```
$ docker exec -it <docker-container-id> bash
# cd athrill-device/device/sample/grpc
# bash ruby_proxy.bash server/ruby/sample_server.rb 
```

* 最初の docker 端末上でデバイスを実行します。

```
# cd /root/workspace/athrill-device/demo/sample/v850
# make clean ; make
# athrill-run
```

athrill の起動に成功すると、以下の起動ログの出力後、実行待ちになります。

```
OK: found device_config.txt
OK: found memory.txt
OK: found test_main.elf
core id num=1
ROM : START=0x0 SIZE=512
RAM : START=0x3ff7000 SIZE=512
RAM : START=0x5ff7000 SIZE=512
DEV : START=0xff100000 SIZE=2
WARNING: unknown memory type=#DEV
RAM : START=0xff300000 SIZE=4
MEMSETTING SET CACHE RIGION:addr=0xff300000 size=4 [KB]
ELF SET CACHE RIGION:addr=0x0 size=5 [KB]
Elf loading was succeeded:0x0 - 0x17f4 : 5.1012 KB
Elf loading was succeeded:0x17f4 - 0x1e64 : 0.0 KB
Elf loading was succeeded:0x1e64 - 0x2264 : 1.0 KB
ELF SYMBOL SECTION LOADED:index=16
ELF SYMBOL SECTION LOADED:sym_num=89
ELF STRING TABLE SECTION LOADED:index=17
SAMPLE_DEVICE: init from tmori.
DEBUG_FUNC_FT_LOG_SIZE=1024
[DBG>
HIT break:0x0
[NEXT> pc=0x0 vector.S 9
```

この状態で、c コマンドを押下すると、次のようになります。

* クライアント側

```
[NEXT> pc=0x0 vector.S 9
c
[CPU>SAMPLE_DEVICE: Hello world from tmori.
Client received: Hello world
SAMPLE_DEVICE: put8() addr=0xff100000 data=0x48(H)
SAMPLE_DEVICE: put8() addr=0xff100001 data=0x65(e)
SAMPLE_DEVICE: put8() addr=0xff100002 data=0x6c(l)
SAMPLE_DEVICE: put8() addr=0xff100003 data=0x6c(l)
SAMPLE_DEVICE: put8() addr=0xff100004 data=0x6f(o)
SAMPLE_DEVICE: put8() addr=0xff100005 data=0x20( )
SAMPLE_DEVICE: put8() addr=0xff100006 data=0x57(W)
SAMPLE_DEVICE: put8() addr=0xff100007 data=0x6f(o)
SAMPLE_DEVICE: put8() addr=0xff100008 data=0x72(r)
SAMPLE_DEVICE: put8() addr=0xff100009 data=0x6c(l)
SAMPLE_DEVICE: put8() addr=0xff10000a data=0x64(d)
SAMPLE_DEVICE: put8() addr=0xff10000b data=0xa(
)
Hello World
```

* サーバー側

```
"name=world clock=10000"
```

# 変更履歴
* 2021/01/03
  * grpc 連携に対応し、athrillデバイスをgrpcサポート言語で開発できるようになりました
* 2021/01/09
  * [Elixir による grpc 連携のサンプル](https://github.com/toppers/athrill-device/tree/main/device/sample/grpc/elixir) を追加しました
