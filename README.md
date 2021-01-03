# athrill-device
athrill device for external shared library.

# athrill-device 紹介記事
* [仮想 IoT デバイスを Athrill で作る！](https://qiita.com/kanetugu2018/items/5bf890c0945c299eb7f0)

# 開発言語
athrill-device は、基本的に C/C++ で開発することを想定しています。一方で、athrill-device は、様々な開発者に利用していただけるように、様々な言語でも開発できるようにしたいと考えています。そのための対応として、[grpc](https://github.com/grpc/grpc) との連携を行えるようにしました。

具体的には、athrill-device 側から grpc の クライアント RPC API(C++) を呼び出し、サーバー側のプログラム言語でデバイス開発を行います。サーバー側のプログラム言語は grpc がサポートしている言語が対象となりますので、以下の言語で開発できます。

* C++
* C#
* Dart
* Go
* Java
* Kotlin
* Node
* Objective-C
* PHP
* Python
* Ruby
* WebJS
* [Elixir](https://github.com/elixir-grpc/grpc)(google公式サポートではないと思われます)

## grpc を利用したサンプルデバイス
grpc を用いた単純なデバイス例として、[sampleデバイス](https://github.com/toppers/athrill-device/tree/main/device/sample) を公開しています。
本サンプルでは、デバイスのクロック情報を grpc の RPC API の引数で渡して、サーバー側(Ruby)プログラムで参照・デバッグ出力しています。

* RPC proto
 * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/proto/sample.proto
* クライアント側
 * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/client/cpp/sample_client.cc
* サーバー側
 * https://github.com/toppers/athrill-device/blob/main/device/sample/grpc/server/ruby/sample_server.rb
 
# 変更履歴
* 2021/01/03
  * grpc 連携し、athrillデバイスをgrpcサポート言語で開発できるようになりました
