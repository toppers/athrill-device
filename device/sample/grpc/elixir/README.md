# athrill-device Elixir sample

[athrill device](https://github.com/toppers/athrill-device) の Elixir によるサンプル

## 準備・前提条件

[Ruby による grpc 連携のサンプル](https://github.com/toppers/athrill-device#grpc-%E3%82%92%E5%88%A9%E7%94%A8%E3%81%97%E3%81%9F%E3%82%B5%E3%83%B3%E3%83%97%E3%83%AB%E3%83%87%E3%83%90%E3%82%A4%E3%82%B9) での準備が完了していることを前提としています。完了していない方（このページから始める方）は、最低限の作業として次の準備を行ってください。

```shell
$ cd <work_dir※作業フォルダを作ってください>
$ git clone https://github.com/toppers/athrill.git
$ git clone https://github.com/toppers/athrill-target-v850e2m.git
$ git clone https://github.com/toppers/athrill-device.git

$ cd athrill-device
$ cd docker/v850
$ bash create-image-athrill.bash
$ bash run-athrill.bash 

# 以降、`athrill-device-v850-elixir` のコンテナ内での操作

# bash install-athrill.bash
# cd athrill-device/device/sample/build
# bash build.bash all

# cd /root/workspace/athrill-device/demo/sample/v850
# make clean ; make
# athrill-run
# exit
```

## 実行方法 

以下の手順でインストールおよび動作確認できます。

* まず、[Elixir による grpc 利用のサンプル](https://github.com/takasehideki/athrill_device_grpc_ex)をここに `clone` します。

```shell
$ cd <work_dir※作成済みの作業フォルダ>
$ cd athrill-device/sample/grpc/elixir
$ git clone https://github.com/takasehideki/athrill_device_grpc_ex.git
```

* 続けて docker イメージを作成してコンテナを起動します。multi-stage buildを採用していますので、Ruby 向けの準備でビルド済みの `kanetugu2015/athrill-device-v850:v1.0.0` のイメージがローカルに存在することを前提とします。

```shell
$ bash create-image-elixir.bash
$ bash run-athrill.bash

# 以降、`athrill-device-v850-elixir` のコンテナ内での操作
```

* Elixir 側のサーバをビルドして起動します。

```shell
# cd athrill_device_grpc_ex
# mix do deps.get, compile
# mix grpc.server
```

* 別の端末から docker コンテナに入り、デバイスを実行します。

```shell
$ docker exec -it <docker-container-id> bash
# cd athrill-device/demo/sample/v850
# athrill-run
```

athrill の起動に成功すると、以下の起動ログの出力後、実行待ちになります。

```shell
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

```shell
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

* サーバー側のログ（例）

```shell
root@docker-desktop:~/workspace/athrill_device_grpc_ex# mix grpc.server 

16:59:44.209 [info]  Running SampleServer.Endpoint with Cowboy using http://0.0.0.0:50051

16:59:49.115 [info]  Handled by SampleServer.request
"name=world clock=10000"

16:59:49.132 [info]  Response :ok in 15ms

```

## 備考

Ruby のサーバに Elixir のクライアントから接続したい場合には、下記のように実行します。

```shell
root@docker-desktop:~/workspace/athrill_device_grpc_ex# mix run priv/sample_client.ex 
"Start: user=grpc-elixir"
%Example.SampleRequest{clock: 0, name: "grpc-elixir"}

17:04:57.100 [info]  Call request of example.SampleService
%Example.SampleReply{ercd: "324", message: "Hello grpc-elixir"}

17:04:57.226 [info]  Got :ok in 99ms
```

