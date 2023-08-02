# F446_Mbed_Template

## これはなに？

Nucleo f446re を PlatformIO で mbed で開発するときのテンプレートです

## 特徴

- mbedignore が設定してあるのでコンパイルが早い
- PlatformIO でも mbed OS v6.15 が使える

## 注意

mbedignore でコンパイルが省略されている機能を使用するときは`.mbedignore`ファイルを編集してください

## 使い方

1. Use This Template を押して新しいリポジトリを作る
1. clone してくる
1. VSCode で開く
1. 普通の PlatformIO プロジェクトとして使う
2. VSCodeの画面下のターミナルのアイコンを開き、pio system infoを実行して、python executableのパスをコピーする

![image](https://user-images.githubusercontent.com/42294158/193441938-c0cf0162-9662-4487-8e11-4668bc1548a4.png)
![image](https://user-images.githubusercontent.com/42294158/193441947-19f69750-a3dd-46e6-98ae-5aac0fcf7093.png)


1. ターミナルで<python executableのパス> -m pip install jinja2 intelhex future prettytable six jsonschemaする

1. 普通の PlatformIO プロジェクトとして使う
