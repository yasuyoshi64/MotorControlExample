# sdkconfig
Serial flasher config

	Flash size				4MB

Partition Table

	Patition Table			Single factory app (large), no OTA

MotorControlExample

	LED GPIO number			5

SD card PIN configuration

	MOSI GPIO number		17
	MOSO GPIO number		19
	CLK GPIO number			18
	CS GPIO number			16
	SD card sw GPIO number	4

OLED(SSD1306) PIN configuration

	I2C SDA GPIO number		21
	I2C SCL GPIO number		22

MOTOR(BD65496) PIN configuration

	INA GPIO NUMBER			12
	INB GPIO NUMBER			13
	PWM GPIO NUMBER			14

HTTP Server

	WebSocket server support	TRUE

FAT Filesystem support

	Long filename support	Long filename buffer in heap
	OEM Code Page			Japanese (DBCS) (CP932)

Color settings

	Color depth.			1: 1 byte per pixel

Font usage

Enable built-in fonts

	Enable UNSCII 8 (Perfect monospace font)	TRUE

3rd Party Libraries

	QR code library			TRUE

# 機能

## SDカードの内容

### ./config

```
ssid=[Wi-Fi SSID]
pass=[Wi-Fi Password]
```

### ./document

ここにはWeb用のファイルを格納します。

## 使い方

* 「No file」と表示されている場合はSDカードを挿入します。
* 「Wi-Fi Connecting」はWi-Fi接続中です。
* Wi-Fi接続完了後はQRコードが表示されます。QRコードを読み込むとWebのURLが読み込まれ、documentのWebが表示されます。
* QRコードは30秒で消灯します。GPIO0のボタンを押下すると再度30秒表示されます。
* SDカードは電源ON中でも抜き差し可能です。


# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
